#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/Utilities.h"
#include "cinder/MayaCamUI.h"
#include "AntTweakBar.h"
#include "mndlkit/params/PParams.h"
#include "BulletWorld.h"
#include "BulletBird.h"
#include "Cinder-LeapSdk.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace LeapSdk;

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
	BulletBird *mBulletBird;
	BulletBird *mBulletBirdDebug;
	AssimpBird *mAssimpBird;
	AssimpBird *mAssimpBirdDebug;

	mndl::kit::params::PInterfaceGl mParams;
	float mFps;

	// camera
	MayaCamUI mMayaCam;
	bool      mCameraLock;
	float     mCameraFov;
	Vec3f     mCameraEyePoint;
	Vec3f     mCameraCenterOfInterestPoint;
	static const int mStepKey = 3;

	// ragdoll
	Vec3f     mPosition;
	Vec3f     mDirection;
	Vec3f     mNormal;

	// Leap
	uint32_t                mCallbackId;
	LeapSdk::HandMap        mHands;
	LeapSdk::DeviceRef      mLeap;
	void                    onFrame( LeapSdk::Frame frame );
	int                     mActHand;
	Vec3f                   mHandPos;
	Vec3f                   mHandDir;
	Vec3f                   mHandNorm;
};

void BulletRagdollApp::setup()
{
	mBulletBird = 0;
	mBulletBirdDebug = 0;
	mAssimpBird = 0;
	mAssimpBirdDebug = 0;

	gl::enableDepthRead();
	gl::enableDepthWrite();

	setupParams();

	CameraPersp cam;
	cam.setPerspective( mCameraFov, getWindowAspectRatio(), 0.1f, 1000.0f );
	cam.setEyePoint( mCameraEyePoint );
	cam.setCenterOfInterestPoint( mCameraCenterOfInterestPoint );
	mMayaCam.setCurrentCam( cam );

	mBulletWorld.setup();

	AssimpBird assimpBird( );

	// Start device
	mLeap = Device::create();
	mCallbackId = mLeap->addCallback( &BulletRagdollApp::onFrame, this );
	mActHand = -1;
}

// Called when Leap frame data is ready
void BulletRagdollApp::onFrame( Frame frame )
{
	mHands = frame.getHands();

	if( mHands.size())
	{
		if( mActHand == -1 )
		{
			for( HandMap::const_iterator it = mHands.begin(); it != mHands.end(); ++it )
			{
				Hand hand = it->second;

				if( hand.getFingers().size() == 5 )
				{
					mActHand = it->first;
					break;
				}
			}
		}
		else
		{
			HandMap::const_iterator it = mHands.find( mActHand );

			if( it != mHands.end())
			{
				mActHand = mHands.begin()->first;
			}
		}
	}
	else
	{
		mActHand = -1;
	}
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
	mParams.addPersistentParam( "Position" , &mPosition , Vec3f( 0.0f, 10.0f, 0.0f ));
	mParams.addPersistentParam( "Direction", &mDirection, Vec3f( 0.0f,  0.0f, -1.0f ));
	mParams.addPersistentParam( "Normal"   , &mNormal   , Vec3f( 0.0f, -1.0f, 0.0f ));
// 	mParams.addButton( "Spawn", [ this ]()
// 								{
// 									if( mBulletBirdDebug )
// 										mBulletWorld.removeBulletBird( mBulletBirdDebug );
// 									mBulletBirdDebug = mBulletWorld.spawnBulletBird( mPosition * 10 );
// 								} );
	mParams.addButton( "SpawnAssimp", [ this ]()
								{
									if( mAssimpBirdDebug )
										mBulletWorld.removeAssimpBird( mAssimpBirdDebug );
									mAssimpBirdDebug = mBulletWorld.spawnAssimpBird( mPosition * 10 );
								} );
// 	mParams.addButton( "Update", [ this ]()
// 								{
// 									if( mBulletBirdDebug )
// 										mBulletWorld.updateBulletBird( mBulletBirdDebug, mPosition * 10, mDirection.normalized(), mNormal.normalized() );
// 									if( mAssimpBirdDebug )
// 										mBulletWorld.updateAssimpBird( mAssimpBirdDebug, mPosition * 10, mDirection.normalized(), mNormal.normalized() );
// 								} );

	mParams.addPersistentParam( "Hand pos" , &mHandPos , Vec3f( -1, -1, -1 ), "", true );
	mParams.addPersistentParam( "Hand dir" , &mHandDir , Vec3f( -1, -1, -1 ), "", true );
	mParams.addPersistentParam( "Hand norm", &mHandNorm, Vec3f( -1, -1, -1 ), "", true );
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

	case KeyEvent::KEY_l:
		{
			mCameraLock = ! mCameraLock;
		}
		break;
	case KeyEvent::KEY_LEFT:
		{
			mMayaCam.mouseDown( Vec2i( mStepKey, 0 ));
			mMayaCam.mouseDrag( Vec2i( mStepKey, 0 ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0       , 0 ), true, false, false );
		}
		break;
	case KeyEvent::KEY_RIGHT:
		{
			mMayaCam.mouseDown( Vec2i( 0       , 0 ));
			mMayaCam.mouseDrag( Vec2i( 0       , 0 ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( mStepKey, 0 ), true, false, false );
		}
		break;
	case KeyEvent::KEY_UP:
		{
			mMayaCam.mouseDown( Vec2i( 0, mStepKey ));
			mMayaCam.mouseDrag( Vec2i( 0, mStepKey ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0, 0        ), true, false, false );
		}
		break;
	case KeyEvent::KEY_DOWN:
		{
			mMayaCam.mouseDown( Vec2i( 0, 0        ));
			mMayaCam.mouseDrag( Vec2i( 0, 0        ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0, mStepKey ), true, false, false );
		}
		break;
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;

	default:
		mBulletWorld.keyDown( event );
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

	// Update device
	if( mLeap && mLeap->isConnected() )
	{
		mLeap->update();
	}

	if( mActHand != -1 )
	{
		Hand hand = mHands[ mActHand ];

		mHandPos  = hand.getPosition();
		mHandDir  = hand.getDirection();
		mHandNorm = hand.getNormal();

// 		if( ! mBulletBird )
// 			mBulletBird = mBulletWorld.spawnBulletBird( mHandPos );
// 
// 		mBulletWorld.updateBulletBird( mBulletBird, mHandPos, mHandDir, mHandNorm );

		if( ! mAssimpBird )
			mAssimpBird = mBulletWorld.spawnAssimpBird( mHandPos );

		mBulletWorld.updateAssimpBird( mAssimpBird, mHandPos, mHandDir, mHandNorm );
	}
	else
	{
		if( mBulletBird )
		{
			mBulletWorld.removeBulletBird( mBulletBird );
			mBulletBird = 0;
		}

		if( mAssimpBird )
		{
			mBulletWorld.removeAssimpBird( mAssimpBird );
			mAssimpBird = 0;
		}

		mHandPos  = Vec3f( -1, -1, -1 );
		mHandDir  = Vec3f( -1, -1, -1 );
		mHandNorm = Vec3f( -1, -1, -1 );
	}

	if( mBulletBirdDebug )
		mBulletWorld.updateBulletBird( mBulletBirdDebug, mPosition * 10, mDirection.normalized(), mNormal.normalized() );

	if( mAssimpBirdDebug )
		mBulletWorld.updateAssimpBird( mAssimpBirdDebug, mPosition * 10, mDirection.normalized(), mNormal.normalized() );
}

void BulletRagdollApp::draw()
{
	// clear out the window with black
	gl::clear( Colorf( 0.392, 0.392, 0.784 ));

	gl::setViewport( getWindowBounds() );
	gl::setMatrices( mMayaCam.getCamera() );

	gl::enableDepthRead();
	gl::enableDepthWrite();

	mBulletWorld.draw();
	mndl::kit::params::PInterfaceGl::draw();

	if( mBulletBird
	 || mAssimpBird )
	{
		glColor4ub( 255, 0, 0, 255 );
		gl::drawVector( Vec3f::zero(), mHandDir * 3, 1, .5 );
		glColor4ub( 0, 255, 0, 255 );
		gl::drawVector( Vec3f::zero(), mHandNorm * 3, 1, .5 );
	}
}

void BulletRagdollApp::resize()
{
	CameraPersp cam = mMayaCam.getCamera();
	cam.setAspectRatio( getWindowAspectRatio() );
	mMayaCam.setCurrentCam( cam );
}

void BulletRagdollApp::shutdown()
{
	mLeap->removeCallback( mCallbackId );
	mHands.clear();

	mndl::kit::params::PInterfaceGl::save();
}

CINDER_APP_NATIVE( BulletRagdollApp, RendererGl )
