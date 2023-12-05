#include "stdafx.h"
#pragma hdrstop

#include "CameraLook.h"
#include "../xrEngine/Cameramanager.h"
#include "xr_level_controller.h"
#include "actor.h"
#include "object_broker.h"
#include "../xrEngine/GameMtlLib.h"

CCameraLook::CCameraLook(CObject* p, u32 flags ) 
:CCameraBase(p, flags)
{
}

void CCameraLook::Load(LPCSTR section)
{
	inherited::Load		(section);
	style				= csLookAt;
	lim_zoom			= pSettings->r_fvector2	(section,"lim_zoom");
	dist				= (lim_zoom[0]+lim_zoom[1])*0.5f;
	prev_d				= 0.0f;
}

CCameraLook::~CCameraLook()
{
}

void CCameraLook::save(NET_Packet& packet) 
{
	save_data(pitch, packet);
}

void CCameraLook::load(IReader& packet) 
{
	load_data(pitch, packet);
}

void CCameraLook::Update(Fvector& point, Fvector& /**noise_dangle/**/)
{
	vPosition.set		(point);
	Fmatrix mR;
	mR.setHPB			(-yaw,-pitch,-roll);

	vDirection.set		(mR.k);
	vNormal.set			(mR.j);

	if (m_Flags.is(flRelativeLink)){
		parent->XFORM().transform_dir(vDirection);
		parent->XFORM().transform_dir(vNormal);
	}
	UpdateDistance		(point);
}

void CCameraLook::UpdateDistance(const Fvector& point) {
	Fvector invertedDirection;
	invertedDirection.invert(vDirection);

	collide::rq_result result;
	float covariance = VIEWPORT_NEAR * 6.0f;
	g_pGameLevel->ObjectSpace.RayPick(point, invertedDirection, dist + covariance, collide::rqtBoth, result, parent);

	CDB::TRI* hitTriangle = Level().ObjectSpace.GetStaticTris() + result.element;
	bool isPassable = IsTrianglePassable(*hitTriangle);

	if (!isPassable)
		AdjustPositionOnCollision(result, covariance);
	else
		vPosition.mul(vDirection, -prev_d - VIEWPORT_NEAR);

	vPosition.add(point);
}

bool CCameraLook::IsTrianglePassable(const CDB::TRI& triangle) const {
	return triangle.material < GMLib.CountMaterial() &&
		GMLib.GetMaterialByIdx(triangle.material)->Flags.is(SGameMtl::flPassable);
}

void CCameraLook::AdjustPositionOnCollision(const collide::rq_result& collisionResult, float covariance) {
	float d = psCamSlideInert * prev_d + (1.0f - psCamSlideInert) * (collisionResult.range - covariance);
	prev_d = d;
	vPosition.mul(vDirection, -d - VIEWPORT_NEAR);
}

void CCameraLook::Move( int cmd, float val, float factor)
{
	switch (cmd){
	case kCAM_ZOOM_IN:	dist	-= val?val:(rot_speed.z*Device.fTimeDelta);	break;
	case kCAM_ZOOM_OUT:	dist	+= val?val:(rot_speed.z*Device.fTimeDelta);	break;
	case kDOWN:			pitch	-= val?val:(rot_speed.x*Device.fTimeDelta/factor);	break;
	case kUP:			pitch	+= val?val:(rot_speed.x*Device.fTimeDelta/factor);	break;
	case kLEFT:			yaw		-= val?val:(rot_speed.y*Device.fTimeDelta/factor);	break;
	case kRIGHT:		yaw		+= val?val:(rot_speed.y*Device.fTimeDelta/factor);	break;
	}
	if (bClampYaw)		clamp(yaw,lim_yaw[0],lim_yaw[1]);
	if (bClampPitch)	clamp(pitch,lim_pitch[0],lim_pitch[1]);
	clamp			(dist,lim_zoom[0],lim_zoom[1]);	
}

void CCameraLook::OnActivate( CCameraBase* old_cam )
{
	if (old_cam) {
		if (m_Flags.is(flRelativeLink) == old_cam->m_Flags.is(flRelativeLink))
			yaw = (old_cam)->yaw;

		if (m_Flags.is(flKeepPitch))
			pitch = (old_cam)->pitch;
	}
	if (yaw>PI_MUL_2) yaw-=PI_MUL_2;
	if (yaw<-PI_MUL_2)yaw+=PI_MUL_2;
}

#include "../xrEngine/xr_input.h"
#include "visual_memory_manager.h"
#include "actor_memory.h"

int cam_dik = DIK_LSHIFT;

Fvector CCameraLook2::m_cam_offset_r;
Fvector CCameraLook2::m_cam_offset_l;
void CCameraLook2::OnActivate( CCameraBase* old_cam )
{
	CCameraLook::OnActivate( old_cam );
}

void CCameraLook2::Update(Fvector& point, Fvector& noise_dangle) {
	// Calculate dynamic rotations based on noise_dangle
	Fmatrix dynamicRotations = CalculateDynamicRotations(noise_dangle);

	// Combine dynamic rotations with existing camera orientation
	Fmatrix combinedOrientation = CombineOrientations(dynamicRotations);

	// Update camera vectors based on the combined orientation
	UpdateCameraVectors(CalculateCombinedMatrix(combinedOrientation));

	// Apply translation and set camera position
	UpdateCameraPosition(point);
}

Fmatrix CCameraLook2::CalculateDynamicRotations(const Fvector& noiseAngles) {
	Fmatrix rX, rY, rZ;
	rX.rotateX(noiseAngles.x);
	rY.rotateY(-noiseAngles.y);
	rZ.rotateZ(noiseAngles.z);

	Fmatrix dynamicRotations;
	dynamicRotations.mul_43(rY, rX);
	dynamicRotations.mulB_43(rZ);

	return dynamicRotations;
}

Fmatrix CCameraLook2::CombineOrientations(const Fmatrix& dynamicRotations) {
	Fmatrix orientationMatrix;

	return orientationMatrix.mulB_43(dynamicRotations);
}

Fmatrix CCameraLook2::CalculateCombinedMatrix(const Fmatrix& combinedOrientation) {
	Fquaternion orientationQuaternion;
	orientationQuaternion.rotationYawPitchRoll(roll, yaw, pitch);

	Fmatrix orientationMatrix = combinedOrientation;
	orientationMatrix.identity();
	orientationMatrix.rotation(orientationQuaternion);
	orientationMatrix.transpose();

	return orientationMatrix;
}

void CCameraLook2::UpdateCameraVectors(const Fmatrix& orientationMatrix) {
	vDirection.set(orientationMatrix.k);
	vNormal.set(orientationMatrix.j);
}

void CCameraLook2::UpdateCameraPosition(const Fvector& point) {
	Fmatrix transformMatrix;
	transformMatrix.setXYZ(0, -yaw, 0);
	transformMatrix.translate_over(point);

	Fvector cameraOffset = CalculateCameraOffset();
	transformMatrix.transform_tiny(cameraOffset);
	vPosition.set(cameraOffset);

	// Check for collisions and adjust position if necessary
	UpdateDistance(cameraOffset);
}

Fvector CCameraLook2::CalculateCameraOffset() {
	return (psActorFlags.test(AF_RIGHT_SHOULDER)) ? m_cam_offset_r : m_cam_offset_l;
}

void CCameraLook2::Load(LPCSTR section)
{
	CCameraLook::Load		(section);

	m_cam_offset_r = pSettings->r_fvector3(section, "offset_right");
	m_cam_offset_l = pSettings->r_fvector3(section, "offset_left");

	dist = 1.4f;
	prev_d = 0.0f;
}


void CCameraFixedLook::Load	(LPCSTR section)
{
	CCameraLook::Load(section);
	style = csFixed;
}

void CCameraFixedLook::OnActivate(CCameraBase* old_cam)
{
	inherited::OnActivate(old_cam);
	m_current_dir.rotationYawPitchRoll(-pitch, -yaw, -roll);	
	
	Fmatrix	rm;
	Fmatrix	trm;
	Fmatrix brm;
	brm.setXYZ(-pitch, -yaw, -roll);
	trm.rotateX(PI_DIV_2);
	rm.mul(brm, trm);
	rm.getXYZ(pitch, yaw, roll);
	m_final_dir.set(rm);
	pitch	= -pitch;
	yaw		= -yaw;
	roll	= -roll;
}

void CCameraFixedLook::Move	(int cmd, float val, float factor)
{
}

void CCameraFixedLook::Update(Fvector& point, Fvector& noise_dangle)
{
	Fquaternion	new_dir;
	new_dir.slerp		(m_current_dir, m_final_dir, Device.fTimeDelta);	//1 sec
	m_current_dir.set	(new_dir);
	
	Fmatrix	rm;
	rm.rotation			(m_current_dir);
	vPosition.set		(point);
	vDirection.set		(rm.k);
	vNormal.set			(rm.j);

	UpdateDistance		(point);
}

void CCameraFixedLook::Set(float Y, float P, float R)
 {
	inherited::Set(Y, P, R);
	Fmatrix	rm;
	rm.setXYZ			(-P, -Y, -R);	
	m_current_dir.set	(rm);
	m_final_dir.set		(m_current_dir);
}