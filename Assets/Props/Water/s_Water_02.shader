Shader "FluidSim/Water_02"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _Color ("Color", Color) = (1,1,1,1)
        _Skybox ("Skybox", Cube) = "defaulttexture" {}
        _LineColor("Line Color", COLOR) = (0,1,0,1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100
        Cull Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

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
                float3 worldViewDir : TEXCOORD1;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            uniform samplerCUBE _Skybox;

            v2f vert (appdata v)
            {
                v2f o;
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                float4 div = tex2Dlod(_MainTex, float4(o.uv, 0, 0));
                float4 vertexPos = v.vertex + div;
                //float4 vertexPos = v.vertex;
                //vertexPos.y += div.r;
                o.vertex = UnityObjectToClipPos(vertexPos);
                o.worldViewDir = WorldSpaceViewDir(vertexPos);

                return o;
            }

            fixed4 _Color;
            fixed4 frag(v2f i) : SV_Target
            {
                float eps = 0.001;
                float hL = tex2D(_MainTex, float2(i.uv.x + eps, i.uv.y)).y;
                float hR = tex2D(_MainTex, float2(i.uv.x - eps, i.uv.y)).y;
                float hT = tex2D(_MainTex, float2(i.uv.x, i.uv.y - eps)).y;
                float hB = tex2D(_MainTex, float2(i.uv.x, i.uv.y + eps)).y;

                float3 norm = normalize( float3( hL - hR, 2 * eps * 10, hB - hT ) );

                //return 2 * 15 * tex2D(_MainTex, i.uv) * 15 * tex2D(_MainTex, i.uv) * 15 * tex2D(_MainTex, i.uv);
                //return 5 * tex2D(_MainTex, i.uv);
                //float diff = 30 * (abs(hL-hR) + abs(hT-hB));
                //return fixed4(diff, diff, diff,1 );
                //return fixed4(norm, 1);
                //return fixed4(_WorldSpaceLightPos0.xyz, 1);
                //return _Color * saturate(dot(norm, _WorldSpaceLightPos0)) + unity_AmbientSky;
                
                float3 viewDir = normalize(i.worldViewDir);
                float3 reflectVec = reflect(-viewDir,norm);
                float4 reflectCol = texCUBE(_Skybox, reflectVec);
                reflectCol = lerp(reflectCol, 0, pow(dot(norm, viewDir), 1));
                return float4(0.8, 0.9, 1.0,1.0) * reflectCol;
                return float4(0.4, 0.6, 0.6, 1.0) * reflectCol;
                
                //float3 refractVec = refract(-viewDir, norm, 1.2);
                //float3 refVec = lerp(reflectVec, refractVec, length(refractVec));
                //float4 refractCol = texCUBE(_Skybox, refVec);
                //return (reflectCol + refractCol) * 0.5;
                
            }
            ENDCG
        }

        //Pass
        //{
        //    Tags { "RenderType" = "Opaque" }
        //    LOD 100

        //    CGPROGRAM
        //    #pragma target 5.0
        //    #pragma vertex vert
        //    #pragma fragment frag
        //    //几何着色器
        //    #pragma geometry geo
        //    #include "UnityCG.cginc"

        //    fixed4 _LineColor;
        //    struct v2g {
        //        //顶点位置
        //        float4  pos       : POSITION;
        //        //法线
        //        float3  normal    : NORMAL;
        //        //纹理坐
        //        float2  tex0        : TEXCOORD0;
        //    };
        //    struct g2f {
        //        //像素位置
        //        float4  pos       : POSITION;
        //        //纹理
        //        float2  tex0        : TEXCOORD0;
        //    };
        //    
        //    v2g vert(appdata_base v)
        //    {
        //        v2g o;
        //        o.pos = UnityObjectToClipPos(v.vertex);
        //        o.normal = v.normal;
        //        //初始化纹理信息
        //        o.tex0 = float2(0, 0);
        //        return o;
        //    }
        //    [maxvertexcount(3)]
        //    void geo(triangle v2g vg[3], inout LineStream<g2f> ls)
        //    {
        //        for (int i = 0; i < 3; i++)
        //        {
        //            g2f g2f_1;
        //            g2f_1.pos = vg[i].pos;
        //            //初始化纹理
        //            g2f_1.tex0 = float2(0.0f, 0.0f);
        //            ls.Append(g2f_1);
        //        }
        //        ls.RestartStrip();
        //    }
        //    fixed4 frag(g2f input) :COLOR
        //    {
        //        return _LineColor;
        //    }
        //    ENDCG
        //}
    }
}
