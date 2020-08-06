#version 430 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;
uniform sampler2D tex;

in TES_OUT
{
  vec3 vNormal;
  vec3 vPosW;
  vec2 vTexCoords;
} gs_in[];

out GS_OUT
{
    vec3 vNormal;
    vec3 vPosW;
    vec2 vTexCoords;
} gs_out;

void EmitTextureVertex(int index)
{
  gl_Position = gl_in[index].gl_Position;
  gs_out.vNormal = gs_in[index].vNormal;
  gs_out.vTexCoords = gs_in[index].vTexCoords;
  gs_out.vPosW = gs_in[index].vPosW;
  EmitVertex();
}

void main() {

  // pass-through original patch vertices with texture coordinates
  EmitTextureVertex(0);
  EmitTextureVertex(1);
  EmitTextureVertex(2);
  EndPrimitive();

}