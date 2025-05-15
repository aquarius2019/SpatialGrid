#pragma once

#include "unordered_dense.h"

namespace SpatialGrid
{
	using CellIndex = FIntVector;

	struct ElementId
	{
		constexpr ElementId() : Index(0), Version(0) {}
		constexpr ElementId(const uint32_t Idx, const uint32_t Ver) : Index(Idx), Version(Ver) {}
		constexpr bool operator==(const ElementId& Other) const
		{
			return Index == Other.Index && Version == Other.Version;
		}

		uint32_t Index;
		uint32_t Version;
	};

	enum class BoundsType : uint8
	{
		Box,
		Sphere,
	};

	struct SPATIALGRID_API Bounds
	{
		static_assert(std::is_trivially_constructible_v<FVector>);
		Bounds() : Origin(FVector::ZeroVector), Type(BoundsType::Sphere), SphereRadius(0.0f) {}
		
		static Bounds MakeSphere(const FVector& origin, const double radius)
		{
			return Bounds(origin, radius);
		}

		static Bounds MakeBox(const FVector& origin, const FVector& box_extent)
		{
			return Bounds(origin, box_extent);
		}

		FBox GetBox() const;
		double GetRadius() const;
		bool OverlapsSphere(const FVector& sphere_origin, const double sphere_radius) const;
		bool OverlapsBox(const FVector& box_origin, const FVector& box_extent) const;
		bool LineHitPoint(const FVector& start, const FVector& end, const FVector& dir, const FVector& inv_dir,
			FVector& out_hit) const;

		void DebugDraw(const UWorld* world) const;

		FVector Origin;
		
	private:
		Bounds(const FVector& origin, const double radius) : Origin(origin), Type(BoundsType::Sphere), SphereRadius(radius) {}
		Bounds(const FVector& origin, const FVector& box_extent) : Origin(origin), Type(BoundsType::Box), BoxExtent(box_extent) {}
		
		BoundsType Type;
		union { FVector BoxExtent; double SphereRadius; };
	};
}

template <>
struct ankerl::unordered_dense::hash<SpatialGrid::ElementId>
{
	using is_avalanching = void;

	[[nodiscard]] auto operator()(SpatialGrid::ElementId const& id) const noexcept -> uint64_t
	{
		static_assert(std::has_unique_object_representations_v<SpatialGrid::ElementId>);
		return ankerl::unordered_dense::detail::wyhash::hash(&id, sizeof(id));
	}
};

template <>
struct ankerl::unordered_dense::hash<SpatialGrid::CellIndex>
{
	using is_avalanching = void;

	[[nodiscard]] auto operator()(SpatialGrid::CellIndex const& index) const noexcept -> uint64_t
	{
		static_assert(std::has_unique_object_representations_v<SpatialGrid::CellIndex>);
		return ankerl::unordered_dense::detail::wyhash::hash(&index, sizeof(index));
	}
};