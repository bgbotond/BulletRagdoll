#include "CinderBulletDebugDrawer.h"
#include "CinderBullet.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"

CinderBulletDebugDrawer::CinderBulletDebugDrawer()
: mDebugModes( 0 )
{
	setDrawEnable( DT_DrawWireframe       , true );
	setDrawEnable( DT_DrawAabb            , true );
	setDrawEnable( DT_DrawFeaturesText    , true );
	setDrawEnable( DT_DrawContactPoints   , true );
	setDrawEnable( DT_NoDeactivation      , true );
	setDrawEnable( DT_NoHelpText          , true );
	setDrawEnable( DT_DrawText            , true );
	setDrawEnable( DT_ProfileTimings      , true );
	setDrawEnable( DT_EnableSatComparison , true );
	setDrawEnable( DT_DisableBulletLCP    , true );
	setDrawEnable( DT_EnableCCD           , true );
	setDrawEnable( DT_DrawConstraints     , true );
	setDrawEnable( DT_DrawConstraintLimits, true );
	setDrawEnable( DT_FastWireframe       , true );
	setDrawEnable( DT_DrawNormals         , true );
}

CinderBulletDebugDrawer::~CinderBulletDebugDrawer()
{
}

void CinderBulletDebugDrawer::drawLine( const btVector3 &from, const btVector3 &to, const btVector3 &color )
{
	ci::ColorA colorA = ci::ColorA( color.getX(), color.getY(), color.getZ() );

	ci::gl::color( colorA );
	ci::gl::drawLine( CinderBullet::convert( from ), CinderBullet::convert( to ));
}

void CinderBulletDebugDrawer::drawContactPoint( const btVector3 &pointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime, const btVector3 &color )
{
	drawLine( pointOnB, pointOnB + normalOnB * distance, color );
}

void CinderBulletDebugDrawer::reportErrorWarning( const char *warningString )
{
	ci::app::App::get()->console() << warningString << std::endl;
}

void CinderBulletDebugDrawer::draw3dText( const btVector3 &location, const char *textString )
{
	ci::gl::drawString( textString, ci::Vec2f( location.getX(), location.getY() ));
}

void CinderBulletDebugDrawer::setDebugMode( int debugMode )
{
	mDebugModes = (DebugDrawModes) debugMode;
}

int CinderBulletDebugDrawer::getDebugMode() const
{
	return mDebugModes;
}

void CinderBulletDebugDrawer::setDrawEnable( DrawType drawType, bool enable )
{
	btIDebugDraw::DebugDrawModes debugModel = btIDebugDraw::DBG_DrawWireframe;

	switch( drawType )
	{
	case DT_DrawWireframe        : debugModel = btIDebugDraw::DBG_DrawWireframe;        break;
	case DT_DrawAabb             : debugModel = btIDebugDraw::DBG_DrawAabb;             break;
	case DT_DrawFeaturesText     : debugModel = btIDebugDraw::DBG_DrawFeaturesText;     break;
	case DT_DrawContactPoints    : debugModel = btIDebugDraw::DBG_DrawContactPoints;    break;
	case DT_NoDeactivation       : debugModel = btIDebugDraw::DBG_NoDeactivation;       break;
	case DT_NoHelpText           : debugModel = btIDebugDraw::DBG_NoHelpText;           break;
	case DT_DrawText             : debugModel = btIDebugDraw::DBG_DrawText;             break;
	case DT_ProfileTimings       : debugModel = btIDebugDraw::DBG_ProfileTimings;       break;
	case DT_EnableSatComparison  : debugModel = btIDebugDraw::DBG_EnableSatComparison;  break;
	case DT_DisableBulletLCP     : debugModel = btIDebugDraw::DBG_DisableBulletLCP;     break;
	case DT_EnableCCD            : debugModel = btIDebugDraw::DBG_EnableCCD;            break;
	case DT_DrawConstraints      : debugModel = btIDebugDraw::DBG_DrawConstraints;      break;
	case DT_DrawConstraintLimits : debugModel = btIDebugDraw::DBG_DrawConstraintLimits; break;
	case DT_FastWireframe        : debugModel = btIDebugDraw::DBG_FastWireframe;        break;
	case DT_DrawNormals          : debugModel = btIDebugDraw::DBG_DrawNormals;          break;
	}

	if( enable )
		mDebugModes |= debugModel;
	else
		mDebugModes &= ~debugModel;
}

bool CinderBulletDebugDrawer::getDrawEnable( DrawType drawType ) const
{
	btIDebugDraw::DebugDrawModes debugModel = btIDebugDraw::DBG_DrawWireframe;

	switch( drawType )
	{
	case DT_DrawWireframe        : debugModel = btIDebugDraw::DBG_DrawWireframe;        break;
	case DT_DrawAabb             : debugModel = btIDebugDraw::DBG_DrawAabb;             break;
	case DT_DrawFeaturesText     : debugModel = btIDebugDraw::DBG_DrawFeaturesText;     break;
	case DT_DrawContactPoints    : debugModel = btIDebugDraw::DBG_DrawContactPoints;    break;
	case DT_NoDeactivation       : debugModel = btIDebugDraw::DBG_NoDeactivation;       break;
	case DT_NoHelpText           : debugModel = btIDebugDraw::DBG_NoHelpText;           break;
	case DT_DrawText             : debugModel = btIDebugDraw::DBG_DrawText;             break;
	case DT_ProfileTimings       : debugModel = btIDebugDraw::DBG_ProfileTimings;       break;
	case DT_EnableSatComparison  : debugModel = btIDebugDraw::DBG_EnableSatComparison;  break;
	case DT_DisableBulletLCP     : debugModel = btIDebugDraw::DBG_DisableBulletLCP;     break;
	case DT_EnableCCD            : debugModel = btIDebugDraw::DBG_EnableCCD;            break;
	case DT_DrawConstraints      : debugModel = btIDebugDraw::DBG_DrawConstraints;      break;
	case DT_DrawConstraintLimits : debugModel = btIDebugDraw::DBG_DrawConstraintLimits; break;
	case DT_FastWireframe        : debugModel = btIDebugDraw::DBG_FastWireframe;        break;
	case DT_DrawNormals          : debugModel = btIDebugDraw::DBG_DrawNormals;          break;
	}

	return( mDebugModes & debugModel ) != 0;
}