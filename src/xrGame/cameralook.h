#pragma once

#include "../xrEngine/CameraBase.h"

class CCameraLook	: public CCameraBase
{
	typedef CCameraBase inherited;

protected:
	Fvector2		lim_zoom;
	float			dist, prev_d;

public:
					CCameraLook		( CObject* p, u32 flags=0);
	virtual			~CCameraLook	( );
	virtual void	Load			(LPCSTR section);
	virtual void	Move			( int cmd, float val=0, float factor=1.0f );

	virtual	void	OnActivate		( CCameraBase* old_cam );
	virtual void	Update			( Fvector& point, Fvector& noise_dangle );

	virtual float	GetWorldYaw		( )	{ return -yaw;	};
	virtual float	GetWorldPitch	( )	{ return pitch; };

	void save(NET_Packet& output_packet) override;
	void load(IReader& input_packet) override;

protected:
	void UpdateDistance(const Fvector& point);

private:
	bool IsTrianglePassable(const CDB::TRI& triangle) const;
	void AdjustPositionOnCollision(const collide::rq_result& collisionResult, float covariance);
};

class CCameraLook2 : public CCameraLook
{
	Fmatrix CalculateDynamicRotations(const Fvector& noiseAngles);
	Fmatrix CombineOrientations(const Fmatrix& dynamicRotations);
	Fmatrix CalculateCombinedMatrix(const Fmatrix& combinedOrientation);
	void UpdateCameraVectors(const Fmatrix& orientationMatrix);
	void UpdateCameraPosition(const Fvector& point);
	Fvector CalculateCameraOffset();

public:
	static Fvector	m_cam_offset_r;
	static Fvector	m_cam_offset_l;

	CCameraLook2(CObject* p, u32 flags = 0) : CCameraLook(p, flags) {};

	virtual			~CCameraLook2() {}
	virtual	void	OnActivate(CCameraBase* old_cam);
	virtual void	Update(Fvector& point, Fvector& noise_dangle);
	virtual void	Load(LPCSTR section);
};

class CCameraFixedLook : public CCameraLook
{
	typedef CCameraLook inherited;
public:
					CCameraFixedLook(CObject* p, u32 flags=0) : CCameraLook(p, flags) {};
	virtual			~CCameraFixedLook() {};
	virtual void	Load			(LPCSTR section);
	virtual void	Move			(int cmd, float val=0, float factor=1.0f);
	virtual	void	OnActivate		(CCameraBase* old_cam);
	virtual void	Update			(Fvector& point, Fvector& noise_dangle);
	virtual void	Set				(float Y, float P, float R);
private:
	Fquaternion		m_final_dir;
	Fquaternion		m_current_dir;
};

