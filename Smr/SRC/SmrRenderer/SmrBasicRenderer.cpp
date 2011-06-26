#include "SmrBasicRenderer.h"

// psecific functions for drawing skeletons

double drawCylinder(SMRVector3 _begin, SMRVector3 _end, unsigned int _step, double _sizeBeg, double _sizeEnd)
{
	// angular displacement
	double stepRad = 2*M_PI/_step;
	// temporary points
	SMRVector3 tmp1, tmp2,tmp3,tmp4;
        SMRVector3 noise(0.0001f,0.0001f,0.0001f);
	SMRVector3 unit = _end - _begin + noise;
	double length = unit.norm();
	unit.normalize();
	SMRVector3 ortho = SMRVector3(-unit.m_y, unit.m_x, 0);
	ortho.normalize();
	// draw begin cap
	//tmp1 = ortho * length * _sizeBeg;
	tmp1 = ortho * _sizeBeg;
	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(-unit.m_x,-unit.m_y,-unit.m_z);
	glVertex3d(_begin.m_x,_begin.m_y,_begin.m_z);
	for (unsigned int i = 0 ; i <=_step ; i++){
		tmp2 = tmp1;
		SMRQuaternion quat(unit,double(i*stepRad));
		//quat = RotationQuaternion(unit,double(i*stepDeg) );
		quat.rotate(tmp2);
		tmp2 = tmp2 + _begin;
		glVertex3d(tmp2.m_x,tmp2.m_y,tmp2.m_z);
	}
	glEnd();
	// draw cylinder
	//tmp1 = ortho * length * _sizeBeg;
	tmp1 = ortho * _sizeBeg;
	tmp3 = ortho * length * _sizeEnd;
	glBegin(GL_TRIANGLE_STRIP);
	glNormal3d(ortho.m_x,ortho.m_y,ortho.m_z);
	glVertex3d(_begin.m_x + tmp1.m_x,_begin.m_y + tmp1.m_y,_begin.m_z + tmp1.m_z);
	glVertex3d(_end.m_x + tmp3.m_x,_end.m_y + tmp3.m_y,_end.m_z + tmp3.m_z);
	for (unsigned int i = 0 ; i <=_step ; i++){
		tmp2 = tmp1;
		tmp4 = tmp3;
		SMRQuaternion quat(unit,double(i*stepRad));
		//quat = RotationQuaternion(unit,double(i*stepDeg) );
		quat.rotate(tmp2);
		quat.rotate(tmp4);
		SMRVector3 tmp5 = tmp2;tmp5.normalize();
		glNormal3d(tmp5.m_x,tmp5.m_y,tmp5.m_z);
		glVertex3d(_begin.m_x + tmp2.m_x,_begin.m_y + tmp2.m_y,_begin.m_z + tmp2.m_z);
		glVertex3d(_end.m_x + tmp4.m_x,_end.m_y + tmp4.m_y,_end.m_z + tmp4.m_z);
	}
	glEnd();
	// draw end cap
	tmp1 = ortho * length * _sizeEnd;
	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(unit.m_x,unit.m_y,unit.m_z);
	glVertex3d(_end.m_x,_end.m_y,_end.m_z);
	for (unsigned int i = 0 ; i <=_step ; i++){
		tmp2 = tmp1;
		SMRQuaternion quat(unit,double(i*stepRad));
		//quat = RotationQuaternion(unit,double(i*stepDeg) );
		quat.rotate(tmp2);
		tmp2 = tmp2 + _end;
		glVertex3d(tmp2.m_x,tmp2.m_y,tmp2.m_z);
	}
	glEnd();

	return (length * _sizeEnd)+0.0001;
}


SMRBasicRenderer::SMRBasicRenderer(SMRSkeleton * _squel) : SMRRenderer(_squel)
{
	m_renderStyle = toon;
	// load shaders
	loadShaders(getFileName("Shaders/toon.vert"),getFileName("Shaders/toon.frag"),"toonShader");
	loadShaders(getFileName("Shaders/gooch.vert"),getFileName("Shaders/gooch.frag"),"goochShader");
}

SMRBasicRenderer::~SMRBasicRenderer()
{
}

void 
SMRBasicRenderer::draw ()
{
	GLuint Shader = 0;
	// load/get shader info
	switch(m_renderStyle)
	{
	case toon:
		Shader = getShader("toonShader");
		break;
	case gooch:
		Shader = getShader("goochShader");
		break;
	}

	bool useShader = isGLSLSupported() && Shader ;
	SMRVector3 tmp, tmp2;
		
	glPushAttrib (GL_POLYGON_BIT);
	glEnable (GL_CULL_FACE);

	// first switch to absolute if needed
	m_skeleton->setMode(ABSOLUTEMODE);	

	glPushMatrix();
	glScaled(m_scaleFactor,m_scaleFactor,m_scaleFactor);

	if (useShader)
	{
		if( GLEW_VERSION_2_0 )
			glUseProgram( Shader );
		else if( GLEW_VERSION_1_5 ) 
			glUseProgramObjectARB( Shader );

		glPolygonMode (GL_BACK, GL_FILL);
		glCullFace (GL_FRONT);
		double size = 0.0001 ;
		for(unsigned int i = 0; i < m_skeleton->getNumJoints(); ++i ) {
			if( m_skeleton->getJoint(i)->getParentName()!="") {
				tmp = m_skeleton->getJoint(i)->getPosition();
				tmp2 = m_skeleton->getJointByName(m_skeleton->getJoint(i)->getParentName())->getPosition();
				size  = drawCylinder(tmp2,tmp,16,size,0.1f);
				if (m_skeleton->getJoint(i)->isEndJoint() )
				{	
					SMRVector3 end = m_skeleton->getJoint(i)->getEndLength();
					//m_skeleton->getJoint(i)->getOrientation().rotate( end );
					//drawCylinder(tmp,tmp+end,16,size,0.0f);
					drawCylinder(tmp,end,16,size,0.0f);
        }
			}
		}
		if( GLEW_VERSION_2_0 )
			glUseProgram( 0 );
		else if( GLEW_VERSION_1_5 ) 
			glUseProgramObjectARB( 0 );

		/*
		* draw back-facing polygons as red lines
		*/

		/* disable lighting for outlining */
		glPushAttrib (GL_LIGHTING_BIT | GL_LINE_BIT | GL_DEPTH_BUFFER_BIT);
		glDisable (GL_LIGHTING);

		glPolygonMode (GL_FRONT, GL_LINE);
		glCullFace (GL_BACK);

		glDepthFunc (GL_LEQUAL);
		glLineWidth (5.0f);


		/* draw wire object */
		glColor3f (0.05f, 0.0f, 0.0f);
		{double size = 0.0001 ;
		for(unsigned int i = 0; i < m_skeleton->getNumJoints(); ++i ) {
			if( m_skeleton->getJoint(i)->getParentName()!="") {
				tmp = m_skeleton->getJoint(i)->getPosition();
				tmp2 = m_skeleton->getJointByName(m_skeleton->getJoint(i)->getParentName())->getPosition();
				size  = drawCylinder(tmp2,tmp,16,size,0.1f);
				glPushMatrix();
				  glTranslated(tmp2.X(),tmp2.Y(),tmp2.Z());
				  SMRQuaternion orient = m_skeleton->getJoint(i)->getOrientation();
				  SMRVector3 axis = orient.getRotationAxis();
				  double angle = orient.getRotationAngle();
				  double axisi[3] = {axis.X(),axis.Y(),axis.Z()};
				  glRotated(angle*180/M_PI,axis.X(),axis.Y(),axis.Z());
				  //drawAxis(4.0);
				glPopMatrix();
				if (m_skeleton->getJoint(i)->isEndJoint() )
				{			
					SMRVector3 end = m_skeleton->getJoint(i)->getEndLength();
					//m_skeleton->getJoint(i)->getOrientation().rotate( end );
					//drawCylinder(tmp,tmp+end,16,size,0.0f);
          drawCylinder(tmp,end,16,size,0.0f);
				}
			}
		}
		}
		/* GL_LIGHTING_BIT | GL_LINE_BIT | GL_DEPTH_BUFFER_BIT */
		glPopAttrib ();
		/* GL_POLYGON_BIT */
		glPopAttrib ();
	}
	//else
	{
		//Draw each bone
		glBegin( GL_LINES );
		for(unsigned int i = 0; i < m_skeleton->getNumJoints(); ++i ) {
			if( m_skeleton->getJoint(i)->getParentName()!="") {
				tmp = m_skeleton->getJoint(i)->getPosition();
				tmp2 = m_skeleton->getJointByName(m_skeleton->getJoint(i)->getParentName())->getPosition();
				glVertex3d( tmp2.m_x,tmp2.m_y ,tmp2.m_z);
				glVertex3d( tmp.m_x,tmp.m_y,tmp.m_z );
			}
		}
		glEnd();
	}
	glPopMatrix();
}

