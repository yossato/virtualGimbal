/*************************************************************************
*  Software License Agreement (BSD 3-Clause License)
*  
*  Copyright (c) 2019, Yoshiaki Sato
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
*  
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*  
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#pragma OPENCL EXTENSION cl_khr_fp64: enable
__constant sampler_t samplerLN = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_LINEAR;



// __kernel void color_shift(
//    const image2d_t src,
//    float shift_x,
//    float shift_y,
//    __global uchar* dst,
//    int dst_step, int dst_offset, int dst_rows, int dst_cols)
// {
//    int x = get_global_id(0);
//    int y = get_global_id(1);
//    if (x >= dst_cols) return;
//    int dst_index = mad24(y, dst_step, mad24(x, (int)sizeof(dstT)*4, dst_offset));
//    __global dstT *dstf = (__global dstT *)(dst + dst_index);
//    float2 coord = (float2)((float)x+0.5f+shift_x, (float)y+0.5f+shift_y);
//    // dstf.x = (dstT)read_imageui(src, samplerLN, coord).x;
//    uint4 pixel = read_imageui(src, samplerLN, coord);
//    dstf.x = pixel.x;
//    dstf.y = pixel.y;
//    dstf.z = pixel.z;
//    dstf[3] = pixel[3];
// }

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
   return 0.5*fabs((p1.x-p3.x)*(p2.y-p3.y)-(p2.x-p3.x)*(p1.y-p3.y));
}

float3 baryventrid_coordinate(float2 pt, float2 p1, float2 p2, float2 p3)
{
   float total = triangle_area(p1,p2,p3);
   float a1 = triangle_area(pt,p2,p3);
   float a2 = triangle_area(p1,pt,p3);
   float a3 = triangle_area(p1,p2,pt);
   return (float3)(a1,a2,a3)/total;
}

float triangle_sign (float2 p1, float2 p2, float2 p3)
{
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

bool point_in_triangle (float2 pt, float2 v1, float2 v2, float2 v3)
{
    bool b1, b2, b3;
 
    b1 = triangle_sign(pt, v1, v2) < 0.0f;
    b2 = triangle_sign(pt, v2, v3) < 0.0f;
    b3 = triangle_sign(pt, v3, v1) < 0.0f;
      
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
   float2 p,                              // UV coordinate position in a image.
   float zoom_ratio,
   __constant float* rotation_matrix,       // Rotation Matrix in each rows.
   float k1, float k2,float p1, float p2, // Distortion parameters.
   float2 f, float2 c
){
   float2 x1 = (p-c)/f;// (float2)((u - cx)/fx,(v-cy)/fy,1.f);
   float r2 = dot(x1,x1);
   float2 x2 = x1*(1.f + k1*r2+k2*r2*r2);
   x2 += (float2)(2.f*p1*x1.x*x1.y+p2*(r2+2.f*x1.x*x1.x), p1*(r2+2.f*x1.y*x1.y)+2.f*p2*x1.x*x1.y);
   //折り返しの話はとりあえずスキップ

   float3 x3 = (float3)(x2.x,x2.y,1.0);
   __constant float* R = rotation_matrix + convert_int( 9*p.y);
   float3 XYZ = (float3)(R[0] * x3.x + R[1] * x3.y + R[2] * x3.z,
                         R[3] * x3.x + R[4] * x3.y + R[5] * x3.z,
                         R[6] * x3.x + R[7] * x3.y + R[8] * x3.z);
   // float3 XYZ = (float3)(0.99922 * x3.x -0.00527565 * x3.y + 0.0391454 * x3.z,
   //                       0.00500257 * x3.x + 0.999963 * x3.y + 0.00707079 * x3.z,
   //                       -0.0391812 * x3.x  -0.00686944 * x3.y + 0.999209 * x3.z);
   x2 = XYZ.xy / XYZ.z;
  return x2*f*zoom_ratio+c;
}

__kernel void stabilizer_function(
   __read_only image2d_t input, __write_only image2d_t output,
   __constant float* rotation_matrix,       // Rotation Matrix in each rows.
   int src_step, int src_offset,
   float zoom_ratio,
   float k1, float k2,float p1, float p2, // Distortion parameters.
   float fx, float fy, float cx, float cy
)
{
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
   float2 uv0 = warp_undistort(uv0_, zoom_ratio, rotation_matrix, k1, k2, p1, p2, f, c);
   float2 uv1 = warp_undistort(uv1_, zoom_ratio, rotation_matrix, k1, k2, p1, p2, f, c);
   float2 uv2 = warp_undistort(uv2_, zoom_ratio, rotation_matrix, k1, k2, p1, p2, f, c);
   float2 uv3 = warp_undistort(uv3_, zoom_ratio, rotation_matrix, k1, k2, p1, p2, f, c);

   
   int2 uvMin = convert_int2(floor(min(min(uv0,uv1),min(uv2,uv3))));
   int2 uvMax = convert_int2(ceil(max(max(uv0,uv1),max(uv2,uv3))));

   // uint4 pixel = read_imageui(input, samplerLN, uv1_);
   // write_imageui(output, convert_int2(uv),pixel);

   // int wrote = 0;
   for(int v= uvMin.y;v<=uvMax.y;++v){
      for(int u=uvMin.x;u<=uvMax.x;++u){
         int2 uvt = (int2)(u,v);
         if(any(uvt >= size)) continue;
         if(any(uvt < 0)) continue;
         

         float2 uw_cam;
         if(point_in_triangle(convert_float2(uvt),uv0,uv1,uv3)){
            float3 ratio = baryventrid_coordinate(convert_float2(uvt),uv0,uv1,uv3);
            uw_cam = uv0_*ratio.x+uv1_*ratio.y+uv3_*ratio.z;
         }else if(point_in_triangle(convert_float2(uvt),uv1,uv2,uv3)){
            float3 ratio = baryventrid_coordinate(convert_float2(uvt),uv1,uv2,uv3);
            uw_cam = uv1_*ratio.x+uv2_*ratio.y+uv3_*ratio.z;
         }else{
            continue;
         }
         uint4 pixel = read_imageui(input, samplerLN, uw_cam);
         write_imageui(output, uvt, pixel);
         // wrote = 1;
      }
   }
   // if(wrote == 0 && get_global_id(1) > 1000 && get_global_id(0) > 100){
   //    printf("uv,%u,%u\nuv0,%f,%f\nuv1,%f,%f\nuv2,%f,%f\nuv3,%f,%f\nuvMin,%d,%d\nuvMax,%d,%d\n",get_global_id(0),get_global_id(1),uv0.x,uv0.y,uv1.x,uv1.y,uv2.x,uv2.y,uv3.x,uv3.y
   //    ,uvMin.x,uvMin.y,uvMax.x,uvMax.y);
   // }
   // uint4 pixel = (uint4)(0,0,0,0);
   // write_imageui(output, (int2)(get_global_id(0),get_global_id(1)),pixel);
}

