Shader "Unlit/YUV2RGBUnlit"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        [Toggle(MIRROR)] _Mirror("Horizontal Mirror", Float) = 0
        [HideInEditor][NoScaleOffset] _YPlane("Y plane", 2D) = "black" {}
        [HideInEditor][NoScaleOffset] _UPlane("U plane", 2D) = "gray" {}
        [HideInEditor][NoScaleOffset] _VPlane("V plane", 2D) = "gray" {}
        [HideInEditor] _Width("Width", Float) = 0
        [HideInEditor] _Height("Height", Float) = 0
        [HideInEditor] _Cx("Cx", Float) = 0
        [HideInEditor] _Cy("Cy", Float) = 0
        [HideInEditor] _Fx("Fx", Float) = 0
        [HideInEditor] _Fy("Fy", Float) = 0
        [HideInEditor] _MinDepth("MinDepth", Float) = 0
        [HideInEditor] _MaxDepth("MaxDepth", Float) = 0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            ColorMaterial AmbientAndDiffuse
            
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #pragma multi_compile __ MIRROR
            #pragma target 3.5

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                uint   id : SV_VertexID;
                float4 color : COLOR;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float4 color : COLOR;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            half3 rgb_to_hsv_no_clip(half3 RGB)
            {
                half3 HSV;
        
                half minChannel, maxChannel;
                if (RGB.x > RGB.y) {
                    maxChannel = RGB.x;
                    minChannel = RGB.y;
                }
                else {
                    maxChannel = RGB.y;
                    minChannel = RGB.x;
                }
                
                if (RGB.z > maxChannel) maxChannel = RGB.z;
                if (RGB.z < minChannel) minChannel = RGB.z;
        
                HSV.xy = 0;
                HSV.z = maxChannel;
                half delta = maxChannel - minChannel;             //Delta RGB value
                if (delta != 0) {                    // If gray, leave H  S at zero
                    HSV.y = delta / HSV.z;
                    half3 delRGB;
                    delRGB = (HSV.zzz - RGB + 3*delta) / (6.0*delta);
                    if      ( RGB.x == HSV.z ) HSV.x = delRGB.z - delRGB.y;
                    else if ( RGB.y == HSV.z ) HSV.x = ( 1.0/3.0) + delRGB.x - delRGB.z;
                    else if ( RGB.z == HSV.z ) HSV.x = ( 2.0/3.0) + delRGB.y - delRGB.x;
                }
                return (HSV);
            }

            sampler2D _YPlane;
            sampler2D _UPlane;
            sampler2D _VPlane;

            float _Width;
            float _Height;
            float _Cx;
            float _Cy;
            float _Fx;
            float _Fy;
            float _MinDepth;
            float _MaxDepth;

            half3 yuv2rgb(half3 yuv)
            {
                // The YUV to RBA conversion, please refer to: http://en.wikipedia.org/wiki/YUV
                // Y'UV420p (I420) to RGB888 conversion section.
                half y_value = yuv[0];
                half u_value = yuv[1];
                half v_value = yuv[2];
                half r = y_value + 1.370705 * (v_value - 0.5);
                half g = y_value - 0.698001 * (v_value - 0.5) - (0.337633 * (u_value - 0.5));
                half b = y_value + 1.732446 * (u_value - 0.5);
                return half3(r, g, b);
            }

            v2f vert (appdata v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_OUTPUT(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                o.uv = v.uv;

                float row = v.id / _Width;
                row = floor(row);
                float col = fmod(v.id, _Width);
                col = floor(col);

                float2 uv_depth;
                uv_depth.x = (col / _Width) * 0.5 + 0.5;
                uv_depth.y = row / _Height;

                half3 yuv_depth;
                yuv_depth.x = tex2Dlod(_YPlane, float4(uv_depth.xy, 0, 0)).r;
                yuv_depth.y = tex2Dlod(_UPlane, float4(uv_depth.xy, 0, 0)).r;
                yuv_depth.z = tex2Dlod(_VPlane, float4(uv_depth.xy, 0, 0)).r;
                half3 rgb_depth = yuv2rgb(yuv_depth);
                half3 hsv_depth = rgb_to_hsv_no_clip(rgb_depth);

                float depth = (hsv_depth.x * (_MaxDepth - _MinDepth) + _MinDepth) * 0.001;

                float threshold_check = ceil(saturate(hsv_depth.z - 0.8));

                v.vertex.x = (depth * (col - _Cx) / _Fx) * threshold_check;
                v.vertex.y = (-depth * (row - _Cy) / _Fy) * threshold_check;
                v.vertex.z = (depth) * threshold_check;

                o.vertex = UnityObjectToClipPos(v.vertex);

                float2 uv_color;
                uv_color.x = (col / _Width) * 0.5;
                uv_color.y = row / _Height;

                half3 yuv_color;
                yuv_color.x = tex2Dlod(_YPlane, float4(uv_color.xy, 0, 0)).r;
                yuv_color.y = tex2Dlod(_UPlane, float4(uv_color.xy, 0, 0)).r;
                yuv_color.z = tex2Dlod(_VPlane, float4(uv_color.xy, 0, 0)).r;
                o.color = float4(yuv2rgb(yuv_color).xyz * threshold_check, threshold_check);

                return o;
            }

            fixed3 frag (v2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
}
