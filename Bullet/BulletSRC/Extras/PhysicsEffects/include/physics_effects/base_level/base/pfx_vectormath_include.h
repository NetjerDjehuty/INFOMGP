/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef _SCE_PFX_VECTORMATH_INCLUDE_H
#define _SCE_PFX_VECTORMATH_INCLUDE_H

// If you want to use the free/open sourced vectormath, you need to
// put codes in the include/vecmath folder and set following define.
#define SCE_PFX_USE_FREE_VECTORMATH

// If you want to use vectomath with SIMD,
// following define is needed.
//#define SCE_PFX_USE_SIMD_VECTORMATH

// This option enables to replace original implementation with 
// vector geometry library.
//#define SCE_PFX_USE_GEOMETRY

// vectormath include
#ifdef SCE_PFX_USE_FREE_VECTORMATH
	// ARA begin insert new code
	#if defined(SCE_PFX_USE_SIMD_VECTORMATH) && defined(__ANDROID__) && defined(__ARM_NEON__)
		// For Android targets supporting NEON instructions,
		// use NEON-optimized vector math library
		#include "../../../vecmath/neon/vectormath_aos.h"
		#include "../../../vecmath/neon/floatInVec.h"
	#else
	// ARA end
		// use standard free vector math library
		#include "../../../vecmath/std/vectormath_aos.h"
		#include "../../../vecmath/std/floatInVec.h"
	// ARA begin insert new code 
	#endif
	// ARA end
	#define SCE_VECTORMATH_AOS_VECTOR_ARG 
	#define SCE_VECTORMATH_AOS_MATRIX_ARG 

	namespace sce {
	namespace PhysicsEffects {
	typedef Vectormath::Aos::Point3     PfxPoint3;
	typedef Vectormath::Aos::Vector3    PfxVector3;
	typedef Vectormath::Aos::Vector4    PfxVector4;
	typedef Vectormath::Aos::Quat       PfxQuat;
	typedef Vectormath::Aos::Matrix3    PfxMatrix3;
	typedef Vectormath::Aos::Matrix4    PfxMatrix4;
	typedef Vectormath::Aos::Transform3 PfxTransform3;
	typedef Vectormath::floatInVec		PfxFloatInVec;
	typedef Vectormath::boolInVec		PfxBoolInVec;
	} //namespace PhysicsEffects
	} //namespace sce
#else
	#include <vectormath.h>
	#define SCE_GEOMETRY_USE_SCALAR_MATH
	#define SCE_VECTORMATH_AOS_VECTOR_ARG SCE_VECTORMATH_SCALAR_AOS_VECTOR_ARG
	#define SCE_VECTORMATH_AOS_MATRIX_ARG SCE_VECTORMATH_SCALAR_AOS_MATRIX_ARG

	namespace sce {
	namespace PhysicsEffects {
	typedef sce::Vectormath::Scalar::Aos::Point3     PfxPoint3;
	typedef sce::Vectormath::Scalar::Aos::Vector3    PfxVector3;
	typedef sce::Vectormath::Scalar::Aos::Vector4    PfxVector4;
	typedef sce::Vectormath::Scalar::Aos::Quat       PfxQuat;
	typedef sce::Vectormath::Scalar::Aos::Matrix3    PfxMatrix3;
	typedef sce::Vectormath::Scalar::Aos::Matrix4    PfxMatrix4;
	typedef sce::Vectormath::Scalar::Aos::Transform3 PfxTransform3;
	typedef sce::Vectormath::Scalar::floatInVec		PfxFloatInVec;
	typedef sce::Vectormath::Scalar::boolInVec		PfxBoolInVec;
	} //namespace PhysicsEffects
	} //namespace sce

	#ifdef SCE_PFX_USE_GEOMETRY
	#include <sce_geometry.h>
	namespace sce {
	namespace PhysicsEffects {
	typedef sce::Geometry::Aos::Line	PfxGeomLine;
	typedef sce::Geometry::Aos::Segment	PfxGeomSegment;
	typedef sce::Geometry::Aos::Plane	PfxGeomPlane;
	typedef sce::Geometry::Aos::Sphere	PfxGeomSphere;
	typedef sce::Geometry::Aos::Capsule	PfxGeomCapsule;
	typedef sce::Geometry::Aos::Bounds	PfxGeomBounds;
	typedef sce::Geometry::Aos::Aabb	PfxGeomAabb;
	typedef sce::Geometry::Aos::Obb		PfxGeomObb;
	} //namespace PhysicsEffects
	} //namespace sce
	#endif
#endif

#endif // _SCE_PFX_VECTORMATH_INCLUDE_H
