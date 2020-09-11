Shader "Unlit/PointCloud2RGB"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        [HideInEditor] _ColorWidth("Width", Float) = 0
        [HideInEditor] _ColorHeight("Height", Float) = 0
        [HideInEditor] _DepthWidth("Width", Float) = 0
        [HideInEditor] _DepthHeight("Height", Float) = 0
        [HideInEditor] _MinDepth("MinDepth", Float) = 0
        [HideInEditor] _MaxDepth("MaxDepth", Float) = 0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            sampler2D _MainTex;
            
            float _ColorWidth;
            float _ColorHeight;
            float _DepthWidth;
            float _DepthHeight;
            float _MinDepth;
            float _MaxDepth;
            float _AdjustmentFactor;

            float3 hsv_to_rgb(float3 HSV)
            {
                    float3 RGB = HSV.z;
                
                    float var_h = HSV.x * 6;
                    int var_i = floor(var_h);   // Or ... var_i = floor( var_h )
                    float var_1 = HSV.z * (1.0 - HSV.y);
                    float var_2 = HSV.z * (1.0 - HSV.y * (var_h-var_i));
                    float var_3 = HSV.z * (1.0 - HSV.y * (1-(var_h-var_i)));
                    if      (var_i == 0) { RGB = float3(HSV.z, var_3, var_1); }
                    else if (var_i == 1) { RGB = float3(var_2, HSV.z, var_1); }
                    else if (var_i == 2) { RGB = float3(var_1, HSV.z, var_3); }
                    else if (var_i == 3) { RGB = float3(var_1, var_2, HSV.z); }
                    else if (var_i == 4) { RGB = float3(var_3, var_1, HSV.z); }
                    else                 { RGB = float3(HSV.z, var_1, var_2); }
                
                return (RGB);
            }

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                UNITY_TRANSFER_FOG(o,o.vertex);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float row = i.uv.y;
                float col = i.uv.x;

                float isDepth = round(col);
                float isColor = 1.0 - isDepth;

                float depth = tex2D(_MainTex, float2(saturate((i.uv.x - 0.5) * 2), i.uv.y)).a;
                float hue = depth / _MaxDepth;

                // ceil used to return a black pixel if depth < _MinDepth
                float3 depth_rgb = hsv_to_rgb(float3(hue, 1.0f, 1.0f * ceil(hue)));

                float4 color = tex2D(_MainTex, float2(saturate(i.uv.x * 2.0), i.uv.y)) * isColor + float4(depth_rgb, 1.0) * isDepth;
                color.a = 1.0;

                return color;
            }
            ENDCG
        }
    }
}
