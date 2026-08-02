#version 450

// Procedural triangle: positions and colours come from gl_VertexIndex, so the
// pipeline binds no vertex buffer (GraphicsPipelineDesc's vertex_bindings stay
// null). The point is not the triangle -- it is proving that the *graphics*
// pipeline works under MoltenVK on iOS: shader module creation, spirv-cross
// reflection, dynamic rendering, and a draw reaching a CAMetalLayer drawable.
// The compute path was proven separately by compute_smoke.

layout(location = 0) out vec3 v_color;

// Clip-space, counter-clockwise front face (gfx's convention).
vec2 positions[3] = vec2[](vec2(0.0, -0.6), vec2(-0.6, 0.5), vec2(0.6, 0.5));
vec3 colors[3] =
    vec3[](vec3(1.0, 0.25, 0.3), vec3(0.25, 1.0, 0.45), vec3(0.3, 0.5, 1.0));

void main() {
  gl_Position = vec4(positions[gl_VertexIndex], 0.0, 1.0);
  v_color = colors[gl_VertexIndex];
}
