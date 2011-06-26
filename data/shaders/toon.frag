
varying vec3 lightDir;
varying vec3 normal;

void main( void )
{
	float intensity;
	vec4 color;
	
	//normalizing the lights position to be on the safe side
	//vec3 n = normalize(normal);
	//intensity = dot(normalize(lightDir),n);
	
	color = vec4(0.8,0.5,0.4,1.0);

	gl_FragColor = color;
	
}
