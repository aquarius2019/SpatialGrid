#pragma once

#include "SpatialGridTypes.h"

namespace SpatialGrid
{
	inline static constexpr FVector INVALID_LOCATION = FVector(DBL_MAX, UE::Math::TVectorConstInit{});
	inline static constexpr FVector INVALID_DIRECTION = FVector(0.0, UE::Math::TVectorConstInit{});

	struct QueryResult
	{
		bool BlockingHit       = false;
		FVector Location       = INVALID_LOCATION;
		FVector ImpactPoint    = INVALID_LOCATION;
		FVector ImpactNormal   = INVALID_DIRECTION;
		ElementId ElementId	   = {};
	};
}