#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

varying vec3 color;

void main()
{
    gl_FragColor = vec4(abs(color), 1.0);
}

