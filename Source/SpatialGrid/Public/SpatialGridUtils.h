#pragma once
#include "SpatialGridTypes.h"

namespace SpatialGrid
{
	struct CellRange
	{
		explicit CellRange(const int32 InStep) : Step(FMath::Abs(InStep)) {}
		explicit CellRange(const CellIndex& InStep)
		: Step(FMath::Abs(InStep.X), FMath::Abs(InStep.Y), FMath::Abs(InStep.Z)) {}

		FORCEINLINE int32 Count() const
		{
			return ((Step.X * 2) + 1) * ((Step.Y * 2) + 1) * ((Step.Z * 2) + 1);
		}
		
		template<typename IterFunc>
		void ForEach(IterFunc&& func) const
		{
			for (int z = -Step.Z; z <= Step.Z; ++z)
			{
				for (int y = -Step.Y; y <= Step.Y; ++y)
				{
					for (int x = -Step.X; x <= Step.X; ++x)
					{
						func(CellIndex(x, y, z));
					}
				}
			}
		}

		template<typename IterFunc>
		void ForEach(const CellIndex& offset, IterFunc&& func) const
		{
			for (int z = -Step.Z; z <= Step.Z; ++z)
			{
				for (int y = -Step.Y; y <= Step.Y; ++y)
				{
					for (int x = -Step.X; x <= Step.X; ++x)
					{
						func(CellIndex(x + offset.X, y + offset.Y, z + offset.Z));
					}
				}
			}
		}
		
	private:
		CellIndex Step;
	};
}

namespace SpatialGrid
{
	template<typename GridSemantics>
	static consteval double HalfCellSize()
	{
		return GridSemantics::CellSize * 0.5;
	}

	template<typename GridSemantics>
	static constexpr double HalfDiagonal()
	{
		return HalfCellSize<GridSemantics>() * FMath::Sqrt(3.0);
	}

	template<typename GridSemantics>
	static consteval FVector CellExtent()
	{
		return FVector(HalfCellSize<GridSemantics>(), UE::Math::TVectorConstInit());
	}
	
	FORCEINLINE static CellIndex RoundVecToInt(const FVector& vector)
	{
		return CellIndex(
		FMath::RoundToInt32(vector.X),
		FMath::RoundToInt32(vector.Y),
		FMath::RoundToInt32(vector.Z));
	}

	FORCEINLINE static bool BoxIntersectsSphere(const FBox& Box, const FVector& SphereOrigin, const double SphereRadius)
	{
		return FVector::DistSquared(SphereOrigin, Box.GetClosestPointTo(SphereOrigin)) <= FMath::Square(SphereRadius);
	}
	
	FORCEINLINE static bool BoxIntersectsSphere(const FVector& BoxOrigin, const FVector& BoxExtent,
		const FVector& SphereOrigin, const double SphereRadius)
	{
		return BoxIntersectsSphere(FBox(BoxOrigin - BoxExtent, BoxOrigin + BoxExtent), SphereOrigin, SphereRadius);
	}

	FORCEINLINE static bool BoxIntersectsSphereRadiusSq(const FBox& Box, const FVector& SphereOrigin, const double RadiusSq)
	{
		return FVector::DistSquared(SphereOrigin, Box.GetClosestPointTo(SphereOrigin)) <= RadiusSq;
	}
	
	static bool BoxIntersectsBox(const FVector& a_origin, const FVector& a_extent, const FVector& b_origin, const FVector& b_extent)
	{
		const FVector a_min = a_origin - a_extent;
		const FVector a_max = a_origin + a_extent;
		
		const FVector b_min = b_origin - b_extent;
		const FVector b_max = b_origin + b_extent;

		if (a_min.X > b_max.X || b_min.X > a_max.X)
		{
			return false;
		}

		if (a_min.Y > b_max.Y || b_min.Y > a_max.Y)
		{
			return false;
		}

		if (a_min.Z > b_max.Z || b_min.Z > a_max.Z)
		{
			return false;
		}

		return true;
	}

	static bool BoxIntersectsBox(const FBox& A, const FBox& B)
	{
		if (A.Min.X > B.Max.X || B.Min.X > A.Max.X)
		{
			return false;
		}

		if (A.Min.Y > B.Max.Y || B.Min.Y > A.Max.Y)
		{
			return false;
		}

		if (A.Min.Z > B.Max.Z || B.Min.Z > A.Max.Z)
		{
			return false;
		}

		return true;
	}
	
	static bool LineIntersectsBox(const FBox& box, const FVector& start, const FVector& inv_dir)
	{
		double t_entry = TNumericLimits<double>::Lowest();
		double t_exit  = TNumericLimits<double>::Max();
		
		for (int axis = 0; axis < 3; ++axis)
		{
			const double t1 = (box.Min[axis] - start[axis]) * inv_dir[axis];
			const double t2 = (box.Max[axis] - start[axis]) * inv_dir[axis];
			
			t_entry = FMath::Max(t_entry, FMath::Min(t1, t2));
			t_exit  = FMath::Min(t_exit,  FMath::Max(t1, t2));

			if (t_entry > t_exit)
			{
				return false; // Ray misses the box
			}
		}

		return true;
	}

	static bool LineIntersectsSphere(const FVector& s, const FVector& end, const FVector& dir,
		const FVector& sphere_origin, const double sphere_radius)
	{
		const FVector start_to_ctr = s - sphere_origin;
		const double radius_sq = sphere_radius * sphere_radius;
		
		if (start_to_ctr.SizeSquared() < radius_sq)
		{
			return true;
		}
		
		const double v = dir | (sphere_origin - s);
		const double discriminant = (radius_sq) - ((start_to_ctr | start_to_ctr) - (v * v));
		
		if(discriminant < 0)
		{
			return false;
		}

		const double time = v - FMath::Sqrt(discriminant);

		if(time < 0 || FMath::Square(time) > FVector::DistSquared(s, end))
		{
			return false;
		}

		return true;
	}

	static bool LineBoxHitPoint(const FBox& box, const FVector& start, const FVector& end, const FVector& dir,
		const FVector& inv_dir, FVector& out_hit)
	{
		if (box.IsInside(start))
		{
			out_hit = start;
			return true;	
		}
		
		double t_entry = TNumericLimits<double>::Lowest();
		double t_exit  = TNumericLimits<double>::Max();
		
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			const double t1 = (box.Min[Axis] - start[Axis]) * inv_dir[Axis];
			const double t2 = (box.Max[Axis] - start[Axis]) * inv_dir[Axis];
			
			t_entry = FMath::Max(t_entry, FMath::Min(t1, t2));
			t_exit  = FMath::Min(t_exit,  FMath::Max(t1, t2));

			if (t_entry > t_exit)
			{
				return false; // Ray misses the box
			}
		}

		if (t_entry < 0 || FMath::Square(t_entry) > FVector::DistSquared(start, end))
		{
			return false;
		}
		
		out_hit = start + (dir * t_entry);
		return true;
	}

	static bool LineSphereHitPoint(const FVector& start, const FVector& end_, const FVector& dir,
		const FVector& sphere_origin, const double sphere_radius, FVector& out_hit)
	{
		const FVector start_to_center = start - sphere_origin;
		const double radius_squared = sphere_radius * sphere_radius;
		
		if (start_to_center.SizeSquared() < radius_squared)
		{
			out_hit = start;
			return true;
		}
		
		const double v = dir | (sphere_origin - start);
		const double discriminant = (radius_squared) - ((start_to_center | start_to_center) - (v * v));
		
		if (discriminant < 0)
		{
			return false;
		}

		const double time = v - FMath::Sqrt(discriminant);

		if (time < 0 || FMath::Square(time) > FVector::DistSquared(start, end_))
		{
			return false;
		}

		out_hit = start + (dir * time);
		return true;
	}
}