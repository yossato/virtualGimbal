#pragma OPENCL EXTENSION cl_khr_fp64: enable
__constant sampler_t samplerLN = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_LINEAR;



__kernel void color_shift(
   const image2d_t src,
   float shift_x,
   float shift_y,
   __global uchar* dst,
   int dst_step, int dst_offset, int dst_rows, int dst_cols)
{
   int x = get_global_id(0);
   int y = get_global_id(1);
   if (x >= dst_cols) return;
   int dst_index = mad24(y, dst_step, mad24(x, (int)sizeof(dstT)*4, dst_offset));
   __global dstT *dstf = (__global dstT *)(dst + dst_index);
   float2 coord = (float2)((float)x+0.5f+shift_x, (float)y+0.5f+shift_y);
   // dstf[0] = (dstT)read_imageui(src, samplerLN, coord).x;
   uint4 pixel = read_imageui(src, samplerLN, coord);
   dstf[0] = pixel[0];
   dstf[1] = pixel[1];
   dstf[2] = pixel[2];
   dstf[3] = pixel[3];
}

__kernel void color_shift2(
   __read_only image2d_t input, __write_only image2d_t output,
   float shift_x,
   float shift_y
)
{
   int2 gid = (int2)(get_global_id(0),get_global_id(1));
   int2 gid_input = gid + (int2)(shift_x,shift_y);
   int2 size = get_image_dim(input);
   if(all(gid < size)){
      uint4 pixel = read_imageui(input, samplerLN, gid_input);
      write_imageui(output, gid, pixel);
   }
}

float triangle_area(float2 p1, float2 p2, float2 p3){
   return 0.5*fabs((p1[0]-p3[0])*(p2[1]-p3[1])-(p2[0]-p3[0])*(p1[1]-p3[1]));
}

float3 baryventrid_coordinate(float2 pt, float2 p1, float2 p2, float2 p3)
{
   float total = triangle_area(p1,p2,p3);
   float a1 = triangle_area(pt,p2,p3);
   float a2 = triangle_area(p1,pt,p3);
   float a3 = triangle_area(p1,p2,pt);
   return (float3)(a1,a2,a3)/total;
}

float sign (float2 p1, float2 p2, float2 p3)
{
    return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1]);
}

bool point_in_triangle (float2 pt, float2 v1, float2 v2, float2 v3)
{
    bool b1, b2, b3;
 
    b1 = sign(pt, v1, v2) < 0.0f;
    b2 = sign(pt, v2, v3) < 0.0f;
    b3 = sign(pt, v3, v1) < 0.0f;
      
    return ((b1 == b2) && (b2 == b3));
}

float2 warp(
   float2 p, float2 p_shift
){
   return p + p_shift;
}

float2 warp_zoom(
   float2 p, float2 p_ratio
){
   return p*p_ratio;
}

float2 warp_undistort(
   float2 p_half,                              // UV coordinate position in a image.
   __constant float* rotation_matrix,       // Rotation Matrix in each rows.
   float k1_half, float k2_half,float p1_half, float p2_half, // Distortion parameters.
   float2 f_half, float2 c_half
){
   //OK
   /** double2 pp = convert_double2(p);
   double2 cc = (double2)(c[0],c[1]);
   double2 ff = (double2)(f[0],f[1]);
   double2 value = ((pp-cc)/ff*ff+cc);
   return (float2)(value[0],value[1]);**/
   double2 p = convert_double2(p_half);
   double k1 = convert_double(k1_half);
   double k2 = convert_double(k2_half);
   double p1 = convert_double(p1_half);
   double p2 = convert_double(p2_half);
   double2 f = convert_double2(f_half);
   double2 c = convert_double2(c_half);

   double2 x1 = (p-c)/f;// (float2)((u - cx)/fx,(v-cy)/fy,1.f);
   double r2 = dot(x1,x1);
   double2 x2 = x1*(1.0 + k1*r2+k2*r2*r2);
   x2 += (double2)(2.0*p1*x1[0]*x1[1]+p2*(r2+2.0*x1[0]*x1[0]), p1*(r2+2.0*x1[1]*x1[1])+2.0*p2*x1[0]*x1[1]);
   //折り返しの話はとりあえずスキップ

   double3 x3 = (double3)(x2[0],x2[1],1.0); //NG 

   __constant float* R = rotation_matrix + convert_int( 9*p[1]);
   double3 XYZ = (double3)(convert_double(R[0]) * x3.x + convert_double(R[1]) * x3.y + convert_double(R[2]) * x3.z,
                         convert_double(R[3]) * x3.x + convert_double(R[4]) * x3.y + convert_double(R[5]) * x3.z,
                         convert_double(R[6]) * x3.x + convert_double(R[7]) * x3.y + convert_double(R[8]) * x3.z);
   x2 = XYZ.xy / XYZ.z;
  return convert_float2(x2*f+c);
}

__kernel void stabilizer_function(
   __read_only image2d_t input, __write_only image2d_t output,
   __constant float* rotation_matrix,       // Rotation Matrix in each rows.
   int src_step, int src_offset,
   float k1, float k2,float p1, float p2, // Distortion parameters.
   float fx, float fy, float cx, float cy
)
{

   if(0 == get_global_id(0) && 0 == get_global_id(1)){
      printf("k1:%f k2:%f p1:%f p2:%f fx:%f fy:%f cx:%f cy:%f\n",k1,k2,p1,p2,fx,fy,cx,fy);
   }
   int2 size = get_image_dim(input);
   float2 uv = convert_float2((int2)(get_global_id(0),get_global_id(1)));
   float2 f = (float2)(fx,fy);
   float2 c = (float2)(cx,cy);

   float2 uv0_ = uv;
   float2 uv1_ = uv + (float2)(1,0);
   float2 uv2_ = uv + (float2)(1,1);
   float2 uv3_ = uv + (float2)(0,1);
   // float2 uv0 = warp_zoom(uv0_,(float2)(2.0f,2.0f));//warp_undistort(uv0_, rotation_matrix, k1, k2, p1, p2, f, c);
   // float2 uv1 = warp_zoom(uv1_,(float2)(2.0f,2.0f));//warp_undistort(uv1_, rotation_matrix, k1, k2, p1, p2, f, c);
   // float2 uv2 = warp_zoom(uv2_,(float2)(2.0f,2.0f));//warp_undistort(uv2_, rotation_matrix, k1, k2, p1, p2, f, c);
   // float2 uv3 = warp_zoom(uv3_,(float2)(2.0f,2.0f));//warp_undistort(uv3_, rotation_matrix, k1, k2, p1, p2, f, c);
   float2 uv0 = warp_undistort(uv0_, rotation_matrix, k1, k2, p1, p2, f, c);
   float2 uv1 = warp_undistort(uv1_, rotation_matrix, k1, k2, p1, p2, f, c);
   float2 uv2 = warp_undistort(uv2_, rotation_matrix, k1, k2, p1, p2, f, c);
   float2 uv3 = warp_undistort(uv3_, rotation_matrix, k1, k2, p1, p2, f, c);

   int2 uvMin = convert_int2(round(min(min(uv0,uv1),min(uv2,uv3))));
   int2 uvMax = convert_int2(round(max(max(uv0,uv1),max(uv2,uv3))));

   uint4 pixel = read_imageui(input, samplerLN, uv0);
   write_imageui(output, convert_int2(uv),pixel);

   // for(int v= uvMin[1];v<uvMax[1];++v){
   //    for(int u=uvMin[0];u<uvMax[0];++u){
   //       int2 uvt = (int2)(u,v);
   //       if(any(uvt >= size)) continue;
   //       if(any(uvt < 0)) continue;
         

   //       float2 uw_cam;
   //       if(point_in_triangle(convert_float2(uvt),uv0,uv1,uv3)){
   //          float3 ratio = baryventrid_coordinate(convert_float2(uvt),uv0,uv1,uv3);
   //          uw_cam = uv0_*ratio[0]+uv1_*ratio[1]+uv3_*ratio[2];
   //       }else if(point_in_triangle(convert_float2(uvt),uv1,uv2,uv3)){
   //          float3 ratio = baryventrid_coordinate(convert_float2(uvt),uv1,uv2,uv3);
   //          uw_cam = uv1_*ratio[0]+uv2_*ratio[1]+uv3_*ratio[2];
   //       }else{
   //          continue;
   //       }
   //       uint4 pixel = read_imageui(input, samplerLN, uw_cam);
   //       write_imageui(output, uvt, pixel);
   //    }
   // }
   // uint4 pixel = (uint4)(0,0,0,0);
   // write_imageui(output, (int2)(get_global_id(0),get_global_id(1)),pixel);
}

