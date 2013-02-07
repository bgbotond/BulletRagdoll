#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/Utilities.h"
#include "cinder/MayaCamUI.h"
#include "AntTweakBar.h"
#include "mndlkit/params/PParams.h"
#include "BulletWorld.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class BulletRagdollApp : public AppNative
{
public:
	void setup();
	void mouseDown( MouseEvent event );
	void mouseDrag( MouseEvent event );
	void mouseUp( MouseEvent event );
	void keyDown( KeyEvent event );
	void update();
	void draw();
	void resize();
	void shutdown();

protected:
	void setupParams();

protected:
	BulletWorld mBulletWorld;

	mndl::kit::params::PInterfaceGl mParams;
	float mFps;

	// camera
	MayaCamUI mMayaCam;
	bool      mCameraLock;
	float     mCameraFov;
	Vec3f     mCameraEyePoint;
	Vec3f     mCameraCenterOfInterestPoint;

	// ragdoll
	Vec3f     mPosition;
};

void BulletRagdollApp::setup()
{
	setupParams();

	CameraPersp cam;
	cam.setPerspective( mCameraFov, getWindowAspectRatio(), 0.1f, 1000.0f );
	cam.setEyePoint( mCameraEyePoint );
	cam.setCenterOfInterestPoint( mCameraCenterOfInterestPoint );
	mMayaCam.setCurrentCam( cam );

	mBulletWorld.setup();
}

void BulletRagdollApp::setupParams()
{
	mndl::kit::params::PInterfaceGl::load( "params.xml" );

	mParams = mndl::kit::params::PInterfaceGl( "Parameters", Vec2i( 230, 300 ), Vec2i( 50, 50 ) );
	mParams.addPersistentSizeAndPosition();

	mFps = 0;
	mParams.addParam( "Fps", &mFps, "", true );
	mParams.addSeparator();
	mParams.addText( "Camera" );
	mParams.addPersistentParam( "Lock camera", &mCameraLock, true );
	mParams.addPersistentParam( "Fov", &mCameraFov, 45.f, "min=20 max=180 step=.1" );
	mParams.addPersistentParam( "Eye", &mCameraEyePoint, Vec3f( 0.0f, 1.0f, 10.0f ));
	mParams.addPersistentParam( "Center of Interest", &mCameraCenterOfInterestPoint, Vec3f( 0.0f, 1.0f, 0.0f ));
	mParams.addText( "Ragdoll" );
	mParams.addPersistentParam( "Position", &mPosition, Vec3f( 0.0f, 10.0f, 0.0f ));
	mParams.addButton( "Spawn", [ this ]()
								{
									mBulletWorld.spawnBulletBird( mPosition );
								} );
}

void BulletRagdollApp::mouseDown( MouseEvent event )
{
	if ( mCameraLock )
		mBulletWorld.mouseDown( event, mMayaCam.getCamera());
	else
		mMayaCam.mouseDown( event.getPos() );
}

void BulletRagdollApp::mouseDrag( MouseEvent event )
{
	if ( mCameraLock )
		mBulletWorld.mouseDrag( event, mMayaCam.getCamera());
	else
		mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
}

void BulletRagdollApp::mouseUp( MouseEvent event )
{
	if ( mCameraLock )
		mBulletWorld.mouseUp( event, mMayaCam.getCamera());
}

void BulletRagdollApp::keyDown( KeyEvent event )
{
	switch( event.getCode() )
	{
	case KeyEvent::KEY_f:
		if ( ! isFullScreen() )
		{
			setFullScreen( true );
			if ( mParams.isVisible() )
				showCursor();
			else
				hideCursor();
		}
		else
		{
			setFullScreen( false );
			showCursor();
		}
		break;

	case KeyEvent::KEY_s:
		{
			mndl::kit::params::PInterfaceGl::showAllParams( !mParams.isVisible() );
			if ( isFullScreen() )
			{
				if ( mParams.isVisible() )
					showCursor();
				else
					hideCursor();
			}
			break;
		}

	case KeyEvent::KEY_ESCAPE:
		quit();
		break;

	default:
		break;
	}
}

void BulletRagdollApp::update()
{
	mFps = getAverageFps();

	mBulletWorld.update();

	CameraPersp cam = mMayaCam.getCamera();
	if ( cam.getFov() != mCameraFov )
	{
		cam.setPerspective( mCameraFov, getWindowAspectRatio(), 0.1f, 1000.0f );
		mMayaCam.setCurrentCam( cam );
	}
	if( mCameraLock )
	{
		if( mCameraEyePoint != cam.getEyePoint())
		{
			cam.setEyePoint( mCameraEyePoint );
			mMayaCam.setCurrentCam( cam );
		}
		if( mCameraCenterOfInterestPoint != cam.getCenterOfInterestPoint())
		{
			cam.setCenterOfInterestPoint( mCameraCenterOfInterestPoint );
			mMayaCam.setCurrentCam( cam );
		}
	}
	else
	{
		mCameraEyePoint              = cam.getEyePoint();
		mCameraCenterOfInterestPoint = cam.getCenterOfInterestPoint();
	}
}

void BulletRagdollApp::draw()
{
	// clear out the window with black
	gl::clear( Color::gray( .8 ) );

	gl::setViewport( getWindowBounds() );
	gl::setMatrices( mMayaCam.getCamera() );

	mBulletWorld.draw();
	mndl::kit::params::PInterfaceGl::draw();
}

void BulletRagdollApp::resize()
{
	CameraPersp cam = mMayaCam.getCamera();
	cam.setAspectRatio( getWindowAspectRatio() );
	mMayaCam.setCurrentCam( cam );
}

void BulletRagdollApp::shutdown()
{
	mndl::kit::params::PInterfaceGl::save();
}

CINDER_APP_NATIVE( BulletRagdollApp, RendererGl )
