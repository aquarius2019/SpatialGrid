#pragma once

#include "unordered_dense.h"

namespace SpatialGrid
{
	using CellIndex = FIntVector;

	struct ElementId
	{
		constexpr ElementId() : Index(0), Version(0) {}
		constexpr ElementId(const uint32_t Idx, const uint32_t Gen) : Index(Idx), Version(Gen) {}
		constexpr bool operator==(const ElementId& Other) const
		{
			return Index == Other.Index && Version == Other.Version;
		}

		uint32_t Index;
		uint32_t Version;
	};
}

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