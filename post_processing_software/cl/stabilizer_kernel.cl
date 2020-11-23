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
__constant sampler_t samplerNN = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

typedef struct {
   float zoom;
   float ik1;
   float ik2;
   float ip1;
   float ip2;
   float fx;
   float fy;
   float cx;
   float cy;
} camera_params;

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
   __global float* rotation_matrix,       // Rotation Matrix in each rows.
   float k1, float k2,float p1, float p2, // Distortion parameters.
   float2 f, float2 c
){
   float2 x1 = (p-c)/f;// (float2)((u - cx)/fx,(v-cy)/fy,1.f);
   float r2 = dot(x1,x1);
   float2 x2 = x1*(1.f + k1*r2+k2*r2*r2);
   x2 += (float2)(2.f*p1*x1.x*x1.y+p2*(r2+2.f*x1.x*x1.x), p1*(r2+2.f*x1.y*x1.y)+2.f*p2*x1.x*x1.y);
   //折り返しの話はとりあえずスキップ

   float3 x3 = (float3)(x2.x,x2.y,1.0);
   __global float* R = rotation_matrix + convert_int( 9*p.y);
   float3 XYZ = (float3)(R[0] * x3.x + R[1] * x3.y + R[2] * x3.z,
                         R[3] * x3.x + R[4] * x3.y + R[5] * x3.z,
                         R[6] * x3.x + R[7] * x3.y + R[8] * x3.z);
   x2 = XYZ.xy / XYZ.z;
  return x2*f*zoom_ratio+c;
}

__kernel void stabilizer_function(
   image2d_t input,
   __global  uchar4 *output,
   int output_step, int output_offset, int output_rows, int output_cols, 
   __global float* rotation_matrix,       // Rotation Matrix in each rows.
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
   float2 uv0 = warp_undistort(uv0_, zoom_ratio, rotation_matrix, k1, k2, p1, p2, f, c);
   float2 uv1 = warp_undistort(uv1_, zoom_ratio, rotation_matrix, k1, k2, p1, p2, f, c);
   float2 uv2 = warp_undistort(uv2_, zoom_ratio, rotation_matrix, k1, k2, p1, p2, f, c);
   float2 uv3 = warp_undistort(uv3_, zoom_ratio, rotation_matrix, k1, k2, p1, p2, f, c);

   
   int2 uvMin = convert_int2(floor(min(min(uv0,uv1),min(uv2,uv3))));
   int2 uvMax = convert_int2(ceil(max(max(uv0,uv1),max(uv2,uv3))));

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

         int output_index = mad24(uvt.y, output_cols, uvt.x);
         __global uchar4 *p_out = (__global uchar4 *)(output + output_index);
         *p_out = convert_uchar4(pixel);
      }
   }

}

__kernel void fill_function(
   image2d_t input,
   __global uchar4 *output,
   int output_step, int output_offset, int output_rows, int output_cols,
   __global float* rotation_matrix,       // Rotation Matrix in each rows.
   int rotation_matrix_step, int rotation_matrix_offset,
   __global float* params,
   int params_step, int params_offset,
   int distance
)
{
   int2 size   = get_image_dim(input);
   int2 uv     = (int2)(get_global_id(0),get_global_id(1));
   if(any(uv >= size)) return;

   __global camera_params* p = (__global camera_params*)params;
   float2 f = (float2)(p->fx,p->fy);
   float2 c = (float2)(p->cx,p->cy);

   float2 uv0_ = convert_float2(uv);
   float2 uv1_ = convert_float2(uv + (int2)(1,0));
   float2 uv2_ = convert_float2(uv + (int2)(1,1));
   float2 uv3_ = convert_float2(uv + (int2)(0,1));
   float2 uv0 = warp_undistort(uv0_, p->zoom, rotation_matrix, p->ik1, p->ik2, p->ip1, p->ip2, f, c);
   float2 uv1 = warp_undistort(uv1_, p->zoom, rotation_matrix, p->ik1, p->ik2, p->ip1, p->ip2, f, c);
   float2 uv2 = warp_undistort(uv2_, p->zoom, rotation_matrix, p->ik1, p->ik2, p->ip1, p->ip2, f, c);
   float2 uv3 = warp_undistort(uv3_, p->zoom, rotation_matrix, p->ik1, p->ik2, p->ip1, p->ip2, f, c);

   
   int2 uvMin = convert_int2(floor(min(min(uv0,uv1),min(uv2,uv3))));
   int2 uvMax = convert_int2(ceil(max(max(uv0,uv1),max(uv2,uv3))));

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
         uint4 pixel = read_imageui(input, samplerLN, uw_cam); // TODO:Fix remove a reading every pixel

         int output_index = mad24(uvt.y, output_cols, uvt.x);
         __global uchar4 *p_out = (__global uchar4 *)(output + output_index);
         if(pixel.w > distance)
         {
            pixel.w = distance;
            *p_out = convert_uchar4(pixel);
         }
      }
   }

}

__kernel void interpoloate_function(
   image2d_t past,
   image2d_t future,
   __global uchar4 *output,
   int output_step, int output_offset, int output_rows, int output_cols
)
{
   int2 size   = get_image_dim(past);
   int2 uv     = (int2)(get_global_id(0),get_global_id(1));
   if(any(uv >= size)) return;

   uchar4 past_pixel     = convert_uchar4_sat(read_imageui(past, samplerNN, uv));
   uchar4 future_pixel   = convert_uchar4_sat(read_imageui(future, samplerNN, uv));

   int output_index = mad24(uv.y, output_cols, uv.x);
   __global uchar4 *p_output = (__global uchar4 *)(output + output_index);
   if(past_pixel.w == 0)
   {
      *p_output = past_pixel;
      // (*p_output).w = 255;    // TODO: Is w channel a invalid value? Shoud I care it?
      return;
   }
   else if(future_pixel.w == 0)
   {
      *p_output = future_pixel;
      // (*p_output).w = 255;
      return;
   }else if((past_pixel.w == 255) && (future_pixel.w == 255))  // Invalid pixel. There is no color information in this pixel.
   {
      *p_output = (uchar4)(0,255,0,255);
   }
   else // Valid
   {
      float ratio = convert_float(past_pixel.w) / convert_float(past_pixel.w + future_pixel.w);
      *p_output = convert_uchar4_sat((1.f - ratio)*convert_float4(past_pixel) + ratio*(convert_float4(future_pixel)));
      // (*p_output).w = 255;
      return;
   }

   

}