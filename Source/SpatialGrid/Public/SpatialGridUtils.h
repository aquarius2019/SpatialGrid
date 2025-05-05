#pragma once
#include "SpatialGridTypes.h"

namespace SpatialGrid
{
	struct CellRange
	{
		CellRange(const int32 InStep) : Step(FMath::Abs(InStep)) {}
		CellRange(const CellIndex& InStep) : Step(FMath::Abs(InStep.X), FMath::Abs(InStep.Y), FMath::Abs(InStep.Z)) {}

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
	static constexpr double half_diagonal()
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
	
	static bool BoxIntersectsBox(const FVector& A_Origin, const FVector& A_Extents, const FVector& B_Origin, const FVector& B_Extents)
	{
		const FVector A_Min = A_Origin - A_Extents;
		const FVector A_Max = A_Origin + A_Extents;
		
		const FVector B_Min = B_Origin - B_Extents;
		const FVector B_Max = B_Origin + B_Extents;

		if (A_Min.X > B_Max.X || B_Min.X > A_Max.X)
		{
			return false;
		}

		if (A_Min.Y > B_Max.Y || B_Min.Y > A_Max.Y)
		{
			return false;
		}

		if (A_Min.Z > B_Max.Z || B_Min.Z > A_Max.Z)
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
	
	static bool LineIntersectsBox(const FBox& Box, const FVector& Start, const FVector& InvDir)
	{
		double T_Entry = TNumericLimits<double>::Lowest();
		double T_Exit  = TNumericLimits<double>::Max();
		
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			const double T1 = (Box.Min[Axis] - Start[Axis]) * InvDir[Axis];
			const double T2 = (Box.Max[Axis] - Start[Axis]) * InvDir[Axis];
			
			T_Entry = FMath::Max(T_Entry, FMath::Min(T1, T2));
			T_Exit  = FMath::Min(T_Exit,  FMath::Max(T1, T2));

			if (T_Entry > T_Exit)
			{
				return false; // Ray misses the box
			}
		}

		return true;
	}

	static bool LineIntersectsSphere(const FVector& Start, const FVector& End, const FVector& Dir,
		const FVector& SphereOrigin, const double SphereRadius)
	{
		const FVector StartToCenter = Start - SphereOrigin;
		const double RadiusSq = SphereRadius * SphereRadius;
		
		if (StartToCenter.SizeSquared() < RadiusSq)
		{
			return true;
		}
		
		const double V = Dir | (SphereOrigin - Start);
		const double Discriminant = (RadiusSq) - ((StartToCenter | StartToCenter) - (V * V));
		
		if(Discriminant < 0)
		{
			return false;
		}

		const double Time = V - FMath::Sqrt(Discriminant);

		if(Time < 0 || FMath::Square(Time) > FVector::DistSquared(Start, End))
		{
			return false;
		}

		return true;
	}
	
	static std::optional<FVector> LineBoxHitPoint(const FBox& Box, const FVector& Start, const FVector& End,
		const FVector& Dir, const FVector& InvDir)
	{
		if (Box.IsInside(Start))
		{
			return Start;	
		}
		
		double T_Entry = TNumericLimits<double>::Lowest();
		double T_Exit  = TNumericLimits<double>::Max();
		
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			const double T1 = (Box.Min[Axis] - Start[Axis]) * InvDir[Axis];
			const double T2 = (Box.Max[Axis] - Start[Axis]) * InvDir[Axis];
			
			T_Entry = FMath::Max(T_Entry, FMath::Min(T1, T2));
			T_Exit  = FMath::Min(T_Exit,  FMath::Max(T1, T2));

			if (T_Entry > T_Exit)
			{
				return std::nullopt; // Ray misses the box
			}
		}

		if (T_Entry < 0 || FMath::Square(T_Entry) > FVector::DistSquared(Start, End))
		{
			return std::nullopt;
		}
		
		return Start + (Dir * T_Entry);
	}

	static bool LineBoxHitPoint(const FBox& Box, const FVector& Start, const FVector& End, const FVector& Dir,
		const FVector& InvDir, FVector& OutHit)
	{
		if (Box.IsInside(Start))
		{
			OutHit = Start;
			return true;	
		}
		
		double T_Entry = TNumericLimits<double>::Lowest();
		double T_Exit  = TNumericLimits<double>::Max();
		
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			const double T1 = (Box.Min[Axis] - Start[Axis]) * InvDir[Axis];
			const double T2 = (Box.Max[Axis] - Start[Axis]) * InvDir[Axis];
			
			T_Entry = FMath::Max(T_Entry, FMath::Min(T1, T2));
			T_Exit  = FMath::Min(T_Exit,  FMath::Max(T1, T2));

			if (T_Entry > T_Exit)
			{
				return false; // Ray misses the box
			}
		}

		if (T_Entry < 0 || FMath::Square(T_Entry) > FVector::DistSquared(Start, End))
		{
			return false;
		}
		
		OutHit = Start + (Dir * T_Entry);
		return true;
	}
	
	// returns the intersection point if the line intersects 
	static std::optional<FVector> LineSphereHitPoint(const FVector& start, const FVector& end_, const FVector& dir,
		const FVector& sphere_origin, const double sphere_radius)
	{
		const FVector start_to_center = start - sphere_origin;
		const double radius_squared = sphere_radius * sphere_radius;
		
		if (start_to_center.SizeSquared() < radius_squared)
		{
			return start;
		}
		
		const double v = dir | (sphere_origin - start);
		const double discriminant = (radius_squared) - ((start_to_center | start_to_center) - (v * v));
		
		if (discriminant < 0)
		{
			return std::nullopt;
		}

		const double time = v - FMath::Sqrt(discriminant);

		if (time < 0 || FMath::Square(time) > FVector::DistSquared(start, end_))
		{
			return std::nullopt;
		}

		return start + (dir * time);
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