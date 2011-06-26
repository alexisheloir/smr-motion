#include "SmrSimpleRenderer.h"

SMRSimpleRenderer::SMRSimpleRenderer(SMRSkeleton * _squel) : SMRRenderer(_squel)
{
	// set default colors
	setDefaultColors();
}

void
SMRSimpleRenderer::setDefaultColors()
{
	// default colors
	m_colorR=0.0f;
	m_colorG=0.3f;
	m_colorB=1.0f ;
	m_colorArticulationR=1.0f ;
	m_colorArticulationG=0.0f ;
	m_colorArticulationB=1.0f ;
}

void 
SMRSimpleRenderer::setColors(float jointR,float jointG,float jointB,
				             float articulationR,float articulationG,float articulationB)
{
	m_colorR=jointR;
	m_colorG=jointG;
	m_colorB=jointB ;
	m_colorArticulationR=articulationR ;
	m_colorArticulationG=articulationG ;
	m_colorArticulationB=articulationB ;
}

void 
SMRSimpleRenderer::draw ()
{
	//basic implementation

	//first switch to absolute if needed
	m_skeleton->setMode(ABSOLUTEMODE);
	// Draw each joint
	glPointSize( 3.0f );
	glColor3f( m_colorArticulationR, m_colorArticulationG, m_colorArticulationB );
	SMRVector3 tmp, tmp2;
	glBegin( GL_POINTS );
	for(unsigned int i = 0; i < m_skeleton->getNumJoints(); ++i ) {		
		tmp = m_skeleton->getJoint(i)->getPosition();
		glVertex3d( tmp.m_x,tmp.m_y,tmp.m_z );
	}
	glEnd();

	glPointSize( 1.0f );

	// Draw each bone
	glColor3f( m_colorR, m_colorG, m_colorB );
	glLineWidth(3.0f);
	glBegin( GL_LINES );
	for(unsigned int i = 0; i < m_skeleton->getNumJoints(); ++i ) {
		if( strcmp(m_skeleton->getJoint(i)->getParentName().c_str(),"")) {
			tmp = m_skeleton->getJoint(i)->getPosition();
			tmp2 = m_skeleton->getJointByName(m_skeleton->getJoint(i)->getParentName())->getPosition();
			glVertex3d( tmp2.m_x,tmp2.m_y ,tmp2.m_z);
			glVertex3d( tmp.m_x,tmp.m_y,tmp.m_z );
			if (m_skeleton->getJoint(i)->isEndJoint() )
			{			
				SMRVector3 end = m_skeleton->getJoint(i)->getEndLength();
				//m_skeleton->getJoint(i)->getOrientation().rotate( end );
				glColor3f( m_colorArticulationR, m_colorArticulationG, m_colorArticulationB );
				glVertex3d( tmp.m_x,tmp.m_y,tmp.m_z );
				glVertex3d( end.m_x,end.m_y,end.m_z );
				glColor3f( m_colorR, m_colorG, m_colorB );
			}
			
		}
	}
	glEnd();
	glLineWidth(1.0f);
}

