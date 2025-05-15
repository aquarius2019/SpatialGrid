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
	
	template<typename Semantics, EQueryCacheType>
	struct TQuery;

	template<typename GridSemantics>
	struct TQueryBuilder;
	
	template<typename Semantics, EQueryCacheType CacheType>
	struct TQueryIter
	{
		using Grid		= TSpatialGrid<Semantics>;
		using Cell		= typename Grid::Cell;
		using Element	= typename Grid::Element;
		using QueryType	= TQuery<Semantics, CacheType>;

		TQueryIter(const QueryType* query, const FVector& origin) : Query(query), Origin(origin) {}

		template<typename F>
		void Each(const Grid& grid, F&& func) const
		{
			if (!Query) return;
			
			if constexpr(CacheType == EQueryCacheType::Cached)
			{
				CachedEach(grid, std::forward<F>(func));
			}
			else
			{
				UncachedEach(grid, std::forward<F>(func));
			}
		}

	private:
		const QueryType* Query = nullptr;
		FVector Origin = FVector::ZeroVector;
		
		template<typename F>
		void CachedEach(const Grid& grid, F&& func) const
		{
			const double radius = Query->Radius;
			const double radius_sq = radius * radius;
			const CellIndex offset = grid.LocationToCoordinates(Origin);

			auto scan_element = [this, radius, func=std::forward<F>(func)](const ElementId id, const Element& element)
			{
				if (Semantics::ElementOverlapsSphere(element, Origin, radius))
				{
					func(id, element);
				}
			};
			auto scan_cell = [this, &grid, &scan_element, radius_sq](const Cell& cell)
			{
				if (BoxIntersectsSphereRadiusSq(cell.GetBounds(), Origin, radius_sq))
				{
					cell.ForEachElement(grid, scan_element);
				}	
			};
			
			if (Query->GetCellCount() > grid.NumCells())
			{
				grid.for_each_cell(scan_cell);
				return;
			}
			
			for (const CellIndex& cell_coord : Query->InnerCells)
			{
				if (const Cell* cell = grid.GetCell(cell_coord + offset); cell && cell->HasElements())
				{
					cell->ForEachElement(grid, std::forward<F>(func));
				}
			}

			for (const CellIndex& cell_coord : Query->EdgeCells)
			{
				if (const Cell* cell = grid.GetCell(cell_coord + offset))
				{
					cell->ForEachElement(grid, scan_element);
				}
			}

			for (const CellIndex& cell_coord : Query->OuterCells)
			{
				const Cell* cell = grid.GetCell(cell_coord + offset);

				if (cell && BoxIntersectsSphereRadiusSq(cell->GetBounds(), Origin, radius_sq))
				{
					cell->ForEachElement(grid, scan_element);
				}
			}
		}

		template<typename F>
		void UncachedEach(const TSpatialGrid<Semantics>& grid, F&& func) const
		{
			if (!Query) { return; }
			
			const double radius = Query->Radius;
			const double radius_sq = radius * radius;
			const CellRange cell_range(FMath::RoundToInt32(radius / Semantics::CellSize) + 1);
			const CellIndex offset = grid.LocationToCoordinates(Origin);

			auto scan_element = [this, radius, func=std::forward<F>(func)](const ElementId id, const Element& element)
			{
				if (Semantics::ElementOverlapsSphere(element, Origin, radius))
				{
					func(id, element);
				}
			};
			
			auto scan_cell = [this, &grid, &scan_element, radius_sq](const Cell& cell)
			{
				if (BoxIntersectsSphereRadiusSq(cell.GetBounds(), Origin, radius_sq))
				{
					cell.ForEachElement(grid, scan_element);
				}	
			};
			
			if (cell_range.Count() > grid.NumCells())
			{
				grid.for_each_cell(scan_cell);
			}
			else
			{
				cell_range.ForEach([&](const CellIndex& cell_coord)
				{
					grid.GetCell(cell_coord + offset, scan_cell);
				});
			}
		}
	};
	
	template<typename Semantics, EQueryCacheType CacheType>
	struct TQuery
	{
		using Grid    = TSpatialGrid<Semantics>;
		using Cell    = typename Grid::Cell;
		using Element = typename Grid::Element;

		
		TQuery() = default;
		
		explicit TQuery(const double radius) : Radius(radius) {}
		
		TQuery(TQuery&& other)
		{
			Radius = other.Radius;
			InnerCells = MoveTemp(other.InnerCells);
			EdgeCells  = MoveTemp(other.EdgeCells);
			OuterCells = MoveTemp(other.OuterCells);
		}
		
		TQueryIter<Semantics, CacheType> SetOrigin(const FVector& origin) const
		{
			return TQueryIter<Semantics, CacheType>(this, origin);
		}

		int32 GetCellCount() const
		{
			return CellCount;
		}

	private:
		double Radius = 0;
		int32 CellCount = 0;
		TArray<CellIndex> InnerCells;
		TArray<CellIndex> EdgeCells;
		TArray<CellIndex> OuterCells;
		friend struct TQueryIter<Semantics, CacheType>;
		friend struct TQueryBuilder<Semantics>;
	};

	template<typename Semantics>
	struct TQueryBuilder
	{
		using Self = TQueryBuilder;
		
		Self& SetRadius(const double radius)
		{
			Shape = EQueryShape::Sphere;
			Radius = radius;
			return *this;
		}

		Self& SetBoxExtent(const FVector& extent)
		{
			Shape = EQueryShape::Box;
			Extents = extent;
			return *this;
		}

		template<EQueryCacheType CacheType>
		TQuery<Semantics, CacheType> Build()
		{
			if constexpr(CacheType == EQueryCacheType::Cached)
			{
				switch (Shape)
				{
				case EQueryShape::Box: return {};
				case EQueryShape::Sphere: return BuildSphere();
				default: return {};
				}
			}
			else
			{
				return TQuery<Semantics, EQueryCacheType::UnCached>(Radius);
			}
		}
		
	private:
		EQueryShape Shape = EQueryShape::Sphere;
		double Radius = 0;
		FVector Extents = FVector(0, 0, 0);
		
		TQuery<Semantics, EQueryCacheType::Cached> BuildSphere()
		{
			TQuery<Semantics, EQueryCacheType::Cached> query(Radius);
			
			const int32 bounds = FMath::RoundToInt32(Radius / Semantics::CellSize) + 1;
			constexpr FVector cell_extent = SpatialGrid::CellExtent<Semantics>();
			// Adjust radius to account for worst-case sphere center position
			const double effective_radius_sq = FMath::Square(Radius - SpatialGrid::HalfDiagonal<Semantics>());
			
			CellRange(bounds).ForEach([&](const CellIndex& index)
			{
				const FVector cell_center(index * Semantics::CellSize);
		
				// For each cell, select the corner coordinate that's furthest from origin
				FVector farthest;
				farthest.X = 0. < cell_center.X ? cell_center.X + cell_extent.X : cell_center.X - cell_extent.X;
				farthest.Y = 0. < cell_center.Y ? cell_center.Y + cell_extent.Y : cell_center.Y - cell_extent.Y;
				farthest.Z = 0. < cell_center.Z ? cell_center.Z + cell_extent.Z : cell_center.Z - cell_extent.Z;
				
				if (farthest.SizeSquared() <= effective_radius_sq)
				{
					query.InnerCells.Add(index);
				}
				else if (FMath::Abs(index.X) < bounds && FMath::Abs(index.Y) < bounds && FMath::Abs(index.Z) < bounds)
				{
					query.EdgeCells.Add(index);
				}
				else
				{
					query.OuterCells.Add(index);
				}
			});

			query.CellCount = query.InnerCells.Num() + query.EdgeCells.Num() + query.OuterCells.Num();
			
			return MoveTemp(query);
		}
	};
}
