#pragma once

#include "Grid.h"
#include "SpatialGridUtils.h"

namespace SpatialGrid
{
	enum class EQueryCacheType
	{
		Cached,
		UnCached
	};
	
	template<typename Semantics, EQueryCacheType>
	struct TSphereQuery;

	template<typename GridSemantics>
	struct TSphereQueryBuilder;
	
	template<typename Semantics, EQueryCacheType CacheType>
	struct TQueryIter
	{
		using Grid		= TSpatialGrid<Semantics>;
		using Cell		= typename Grid::Cell;
		using Element	= typename Grid::Element;
		using QueryType	= TSphereQuery<Semantics, CacheType>;

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
				if (element.Bounds.OverlapsSphere(Origin, radius))
				{
					func(id, element);
				}
			};
			auto scan_cell = [this, &grid, &scan_element, radius_sq](const CellIndex&, const Cell& cell)
			{
				if (BoxIntersectsSphereRadiusSq(cell.GetBounds(), Origin, radius_sq))
				{
					cell.ForEachElement(grid, scan_element);
				}	
			};
			
			if (Query->CellCount() > grid.NumCells())
			{
				grid.ForEachCell(scan_cell);
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
		void UncachedEach(const Grid& grid, F&& func) const
		{
			if (!Query) { return; }
			
			const double radius = Query->Radius;
			const double radius_sq = radius * radius;
			const CellRange cell_range(FMath::RoundToInt32(radius / Semantics::CellSize) + 1);
			const CellIndex offset = grid.LocationToCoordinates(Origin);

			auto scan_element = [this, radius, func=std::forward<F>(func)](const ElementId id, const Element& element)
			{
				if (element.Bounds.OverlapsSphere(Origin, radius))
				{
					func(id, element);
				}
			};
			
			auto scan_cell = [this, &grid, &scan_element, radius_sq](const CellIndex&, const Cell& cell)
			{
				if (BoxIntersectsSphereRadiusSq(cell.GetBounds(), Origin, radius_sq))
				{
					cell.ForEachElement(grid, scan_element);
				}	
			};
			
			if (cell_range.Count() > grid.NumCells())
			{
				grid.ForEachCell(scan_cell);
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
	struct TSphereQuery
	{
		explicit TSphereQuery() = default;
		explicit TSphereQuery(const double radius) : Radius(radius) {}
		
		TQueryIter<Semantics, CacheType> SetOrigin(const FVector& origin) const
		{
			return TQueryIter<Semantics, CacheType>(this, origin);
		}
	private:
		double Radius = 0;
		
		friend struct TQueryIter<Semantics, CacheType>;
		friend struct TSphereQueryBuilder<Semantics>;
	};

	template<typename Semantics>
	struct TSphereQuery<Semantics, EQueryCacheType::Cached>
	{
		explicit TSphereQuery() = default;
		explicit TSphereQuery(const double radius) : Radius(radius) {}
		
		TQueryIter<Semantics, EQueryCacheType::Cached> SetOrigin(const FVector& origin) const
		{
			return TQueryIter<Semantics, EQueryCacheType::Cached>(this, origin);
		}
		
		int32 CellCount() const
		{
			return InnerCells.Num() + EdgeCells.Num() + OuterCells.Num();
		}
		
	private:
		double Radius = 0;
		TArray<CellIndex> InnerCells;
		TArray<CellIndex> EdgeCells;
		TArray<CellIndex> OuterCells;
		
		friend struct TQueryIter<Semantics, EQueryCacheType::Cached>;
		friend struct TSphereQueryBuilder<Semantics>;
	};
	
	template<typename Semantics>
	struct TSphereQueryBuilder
	{
		using Self = TSphereQueryBuilder;
		
		Self& SetRadius(const double radius)
		{
			Radius = radius;
			return *this;
		}
		
		template<EQueryCacheType CacheType>
		TSphereQuery<Semantics, CacheType> Build()
		{
			if constexpr(CacheType == EQueryCacheType::Cached)
			{
				return BuildCached();
			}
			else
			{
				return TSphereQuery<Semantics, EQueryCacheType::UnCached>(Radius);
			}
		}
		
	private:
		double Radius = Semantics::CellSize;
		
		TSphereQuery<Semantics, EQueryCacheType::Cached> BuildCached()
		{
			TSphereQuery<Semantics, EQueryCacheType::Cached> query(Radius);
			
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
			
			return MoveTemp(query);
		}
	};
}
