#pragma once

#include "Grid.h"
#include "SpatialGridUtils.h"

namespace SpatialGrid
{
	enum class EQueryShape
	{
		Box,
		Sphere,
	};

	enum class EQueryCacheType
	{
		Cached,
		UnCached
	};
	
	template<typename GridSemantics, EQueryCacheType>
	struct TQuery;

	template<typename GridSemantics>
	struct TQueryBuilder;
	
	template<typename GridSemantics, EQueryCacheType CacheType>
	struct TQueryIter
	{
		using QueryType = TQuery<GridSemantics, CacheType>;

		TQueryIter(const QueryType* query, const FVector& origin) : m_query(query), m_origin(origin) {}

		template<typename IterFunc>
		void each(const TSpatialGrid<GridSemantics>& grid, IterFunc&& func) const
		{
			if constexpr(CacheType == EQueryCacheType::Cached)
			{
				cached_each(grid, std::forward<IterFunc>(func));
			}
			else
			{
				uncached_each(grid, std::forward<IterFunc>(func));
			}
		}

	private:
		const QueryType* m_query = nullptr;
		FVector m_origin = FVector::ZeroVector;
		
		template<typename IterFunc>
		void cached_each(const TSpatialGrid<GridSemantics>& grid, IterFunc&& func) const
		{
			using GridCell    = typename TSpatialGrid<GridSemantics>::Cell;
			using ElementType = typename TSpatialGrid<GridSemantics>::ElementData;
			
			const double radius = m_query->m_radius;
			const double radius_sq = radius * radius;
			const CellIndex offset = grid.LocationToCoordinates(m_origin);

			if (m_query->cell_count() > grid.NumCells())
			{
				grid.for_each_cell([&](const GridCell& cell)
				{
					if (cell.HasElements() && BoxIntersectsSphereRadiusSq(cell.GetBounds(), m_origin, radius_sq))
					{
						cell.ForEachElement(grid, [&](const ElementId id, const ElementType& element)
						{
							if (GridSemantics::ElementOverlapsSphere(element, m_origin, radius))
							{
								func(id, element);
							}
						});
					}
				});

				return;
			}
			
			for (const CellIndex& cell_coord : m_query->m_inner_cells)
			{
				const GridCell* cell = grid.GetCell(cell_coord + offset);
				
				if (cell && cell->HasElements())
				{
					cell->ForEachElement(grid, std::forward<IterFunc>(func));
				}
			}

			for (const CellIndex& cell_coord : m_query->m_edge_cells)
			{
				if (const GridCell* cell = grid.GetCell(cell_coord + offset))
				{
					cell->ForEachElement(grid, [&](const ElementId id, const ElementType& element)
					{
						if (GridSemantics::ElementOverlapsSphere(element, m_origin, radius))
						{
							func(id, element);
						}
					});
				}
			}

			for (const CellIndex& cell_coord : m_query->m_outer_cells)
			{
				const GridCell* cell = grid.GetCell(cell_coord + offset);

				if (cell && BoxIntersectsSphereRadiusSq(cell->GetBounds(), m_origin, radius_sq))
				{
					cell->ForEachElement(grid, [&](const ElementId id, const ElementType& element)
					{
						if (GridSemantics::ElementOverlapsSphere(element, m_origin, radius))
						{
							func(id, element);
						}
					});
				}
			}
		}

		template<typename IterFunc>
		void uncached_each(const TSpatialGrid<GridSemantics>& grid, IterFunc&& func) const
		{
			using GridCell    = typename TSpatialGrid<GridSemantics>::Cell;
			using ElementType = typename TSpatialGrid<GridSemantics>::ElementData;
			
			const double radius = m_query->m_radius;
			const double radius_sq = radius * radius;
			const int32 bounds = FMath::RoundToInt32(radius / GridSemantics::CellSize) + 1;
			const CellIndex offset = grid.LocationToCoordinates(m_origin);
			const CellRange cell_range(bounds);

			if (cell_range.Count() > grid.NumCells())
			{
				grid.for_each_cell([&](const GridCell& cell)
				{
					if (BoxIntersectsSphereRadiusSq(cell.GetBounds(), m_origin, radius_sq))
					{
						cell.ForEachElement(grid, [&](const ElementId id, const ElementType& element)
						{
							if(GridSemantics::ElementOverlapsSphere(element, m_origin, radius))
							{
								func(id, element);
							}
						});
					}
				});

				return;
			}

			cell_range.ForEach([&](const CellIndex& cell_coord)
			{
				const GridCell* cell = grid.GetCell(cell_coord + offset);
				
				if (cell && BoxIntersectsSphereRadiusSq(cell->GetBounds(), m_origin, radius_sq))
				{
					cell->ForEachElement(grid, [&](const ElementId id, const ElementType& element)
					{
						if (GridSemantics::ElementOverlapsSphere(element, m_origin, radius))
						{
							func(id, element);
						}
					});
				}
			});
		}
	};
	
	template<typename GridSemantics, EQueryCacheType CacheType>
	struct TQuery
	{
		TQuery() = default;
		
		TQuery(const double radius) : m_radius(radius) {}
		
		TQuery(TQuery&& other)
		{
			m_radius = other.m_radius;
			m_inner_cells = MoveTemp(other.m_inner_cells);
			m_edge_cells = MoveTemp(other.m_edge_cells);
			m_outer_cells = MoveTemp(other.m_outer_cells);
		}
		
		FORCEINLINE TQueryIter<GridSemantics, CacheType> set_origin(const FVector& location) const
		{
			return TQueryIter<GridSemantics, CacheType>(this, location);
		}

		FORCEINLINE int32 cell_count() const
		{
			if constexpr (CacheType == EQueryCacheType::Cached)
			{
				return m_inner_cells.Num() + m_edge_cells.Num() + m_outer_cells.Num();
			}
			else
			{
				const int32 bounds = FMath::RoundToInt32(m_radius / GridSemantics::CellSize) + 1;
				return FMath::Cube((bounds * 2) + 1);
			}
		}

	private:
		double m_radius = 0;
		
		TArray<CellIndex> m_inner_cells;
		TArray<CellIndex> m_edge_cells;
		TArray<CellIndex> m_outer_cells;

		friend struct TQueryIter<GridSemantics, CacheType>;
		friend struct TQueryBuilder<GridSemantics>;
	};

	template<typename GridSemantics>
	struct TQueryBuilder
	{
		using Self = TQueryBuilder;
		
		Self& radius(const double radius)
		{
			m_shape = EQueryShape::Sphere;
			m_radius = radius;
			return *this;
		}

		Self& box_extent(const FVector& extent)
		{
			m_shape = EQueryShape::Box;
			m_extents = extent;
			return *this;
		}

		template<EQueryCacheType CacheType>
		TQuery<GridSemantics, CacheType> build()
		{
			if constexpr(CacheType == EQueryCacheType::Cached)
			{
				switch (m_shape)
				{
				case EQueryShape::Box: return {};
				case EQueryShape::Sphere: return build_sphere_query();
				}
			}

			return TQuery<GridSemantics, CacheType>(m_radius);
		}
		
	private:
		EQueryShape m_shape = EQueryShape::Sphere;
		double m_radius = 0;
		FVector m_extents = FVector(0, 0, 0);
		
		TQuery<GridSemantics, EQueryCacheType::Cached> build_sphere_query()
		{
			TQuery<GridSemantics, EQueryCacheType::Cached> query(m_radius);
			
			const int32 bounds = FMath::RoundToInt32(m_radius / GridSemantics::CellSize) + 1;
			const FVector cell_extent = FVector(SpatialGrid::HalfCellSize<GridSemantics>());
			// Adjust radius to account for worst-case sphere center position
			const double effective_radius_sq = FMath::Square(m_radius - SpatialGrid::half_diagonal<GridSemantics>());
			
			CellRange(bounds).ForEach([&](const CellIndex& index)
			{
				const FVector cell_center(index * GridSemantics::CellSize);
		
				// For each cell, select the corner coordinate that's furthest from origin
				FVector farthest_point;
				farthest_point.X = 0. < cell_center.X ? cell_center.X + cell_extent.X : cell_center.X - cell_extent.X;
				farthest_point.Y = 0. < cell_center.Y ? cell_center.Y + cell_extent.Y : cell_center.Y - cell_extent.Y;
				farthest_point.Z = 0. < cell_center.Z ? cell_center.Z + cell_extent.Z : cell_center.Z - cell_extent.Z;
				
				if (farthest_point.SizeSquared() <= effective_radius_sq)
				{
					query.m_inner_cells.Add(index);
				}
				else if (FMath::Abs(index.X) < bounds && FMath::Abs(index.Y) < bounds && FMath::Abs(index.Z) < bounds)
				{
					query.m_edge_cells.Add(index);
				}
				else
				{
					query.m_outer_cells.Add(index);
				}
			});
			
			return MoveTemp(query);
		}
	};
}
