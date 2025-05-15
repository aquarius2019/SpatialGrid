#include "SpatialGridTypes.h"

#include "SpatialGridUtils.h"

namespace SpatialGrid
{
	FBox Bounds::GetBox() const
	{
		check(Type == BoundsType::Box);
		return FBox(Origin - BoxExtent, Origin + BoxExtent);
	}

	double Bounds::GetRadius() const
	{
		switch (Type)
		{
			case BoundsType::Box: return BoxExtent.Size();
			case BoundsType::Sphere: return SphereRadius;
		}

		return 0.0;
	}

	bool Bounds::OverlapsSphere(const FVector& sphere_origin, const double sphere_radius) const
	{
		switch (Type)
		{
		case BoundsType::Box: return BoxIntersectsSphere(Origin, BoxExtent, sphere_origin, sphere_radius);
		case BoundsType::Sphere: return FVector::DistSquared(sphere_origin, Origin) <= FMath::Square(SphereRadius + sphere_radius);
		}

		return false;
	}

	bool Bounds::OverlapsBox(const FVector& box_origin, const FVector& box_extent) const
	{
		switch (Type)
		{
		case BoundsType::Box: return BoxIntersectsBox(Origin, BoxExtent, box_origin, box_extent);
		case BoundsType::Sphere: return BoxIntersectsSphere(box_origin, box_extent, Origin, SphereRadius);
		}

		return false;
	}

	bool Bounds::LineHitPoint(const FVector& start, const FVector& end, const FVector& dir, const FVector& inv_dir,
	                          FVector& out_hit) const
	{
		switch (Type)
		{
		case BoundsType::Box: return LineBoxHitPoint(GetBox(), start, end, dir, inv_dir, out_hit);
		case BoundsType::Sphere: return LineSphereHitPoint(start, end, dir, Origin, SphereRadius, out_hit);
		}

		return false;
	}

	void Bounds::DebugDraw(const UWorld* world) const
	{
		check(world);
		
		switch (Type)
		{
			case BoundsType::Box: return DrawDebugBox(world, Origin, BoxExtent, FColor::Blue);
			case BoundsType::Sphere: return DrawDebugSphere(world, Origin, SphereRadius, 8, FColor::Blue);
		}
	}
}
