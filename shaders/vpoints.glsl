#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

attribute vec3 a_position;
//attribute vec3 a_normal;
attribute vec3 a_color;

uniform mat4 uMVMat;
uniform mat4 uPMat;
//uniform mat3 uNMat;

varying vec3 color;

void main() {
    gl_Position = uPMat * uMVMat * vec4(a_position,1.0);
    //color = normalize(abs(a_normal));
    color = a_color;
}

