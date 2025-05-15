#pragma once

#include "Grid.h"
#include "SpatialGridQueryResult.h"
#include "SpatialGridUtils.h"

namespace SpatialGrid
{
	template<typename Semantics>
	struct TLineTrace
	{
		using Grid    = TSpatialGrid<Semantics>;
		using Cell    = typename Grid::Cell;
		using Element = typename Grid::Element;
		using CellSet = ankerl::unordered_dense::set<CellIndex>;
		
		TLineTrace(const FVector& start, const FVector& end)
		: Start(start)
		, End(end)
		, Dir((end - start).GetSafeNormal())
		, InvDir(Dir.Reciprocal())
		, Delta(
			FMath::Abs(Semantics::CellSize * InvDir.X),
			FMath::Abs(Semantics::CellSize * InvDir.Y),
			FMath::Abs(Semantics::CellSize * InvDir.Z))
		, Step(Dir.X > 0 ? 1 : -1, Dir.Y > 0 ? 1 : -1, Dir.Z > 0 ? 1 : -1) {}
		
		TLineTrace(const FVector& start, const FVector& direction, const double length)
		: Start(start)
		, End(start + (direction * length))
		, Dir(direction)
		, InvDir(Dir.Reciprocal())
		, Delta(
			FMath::Abs(Semantics::CellSize * InvDir.X),
			FMath::Abs(Semantics::CellSize * InvDir.Y),
			FMath::Abs(Semantics::CellSize * InvDir.Z))
		, Step(Dir.X > 0 ? 1 : -1, Dir.Y > 0 ? 1 : -1, Dir.Z > 0 ? 1 : -1) {}
		
		template<typename IterFunc>
		void Multi(const TSpatialGrid<Semantics>& grid, IterFunc&& func) const
		{
			// check that line intersects current grid bounds
			FVector hit_point;
			if (!LineBoxHitPoint(grid.GetBounds(), Start, End, Dir, InvDir, hit_point))
			{
				return;
			}

			// TODO: figure out a good algorithm for reserving memory
			CellSet   checked_cells(100);
			CellIndex current_cell = grid.LocationToCoordinates(hit_point);
			const FVector start_cell_origin = grid.CellCenter(current_cell);
			const FVector t1 = ((start_cell_origin - cell_extent) - hit_point) * InvDir;
			const FVector t2 = ((start_cell_origin + cell_extent) - hit_point) * InvDir;
			const CellIndex end_cell = grid.LocationToCoordinates(End);

			FVector t_max = FVector::Max(t1, t2);

			if (hit_point != Start)
			{
				Progress(current_cell, t_max);
			}

			const int32 max_steps = CalculateMaxSteps(hit_point);
			
			for (int32 step = 0; step < max_steps; ++step)
			{
				CheckAll(grid, current_cell, checked_cells, std::forward<IterFunc>(func));

				if (current_cell == end_cell || !grid.IsCellWithinBounds(current_cell))
				{
					break;
				}

				Progress(current_cell, t_max);
			}
		}
		
		QueryResult Single(const Grid& grid) const
		{
			QueryResult result = {};
			
			// check that line intersects current grid bounds
			FVector hit_point;
			
			if (!LineBoxHitPoint(grid.GetBounds(), Start, End, Dir, InvDir, hit_point))
			{
				return result;
			}

			// TODO: figure out a good algorithm for reserving memory
			CellSet   checked_cells(100);
			CellIndex current_cell = grid.LocationToCoordinates(hit_point);
			const FVector start_cell_origin = grid.CellCenter(current_cell);
			const FVector t1 = ((start_cell_origin - cell_extent) - hit_point) * InvDir;
			const FVector t2 = ((start_cell_origin + cell_extent) - hit_point) * InvDir;
			const CellIndex end_cell = grid.LocationToCoordinates(End);

			FVector t_max = FVector::Max(t1, t2);

			if (hit_point != Start)
			{
				Progress(current_cell, t_max);
			}

			const int32 max_steps = CalculateMaxSteps(hit_point);
			
			for(int32 steps = 0; steps < max_steps; ++steps) 
			{
				CheckClosest(grid, current_cell, checked_cells, result);

				if (result.BlockingHit || current_cell == end_cell || !grid.IsCellWithinBounds(current_cell))
				{
					break;
				}

				Progress(current_cell, t_max);
			}
	
			return result;
		}
		
	private:
		FVector Start;
		FVector End;
		FVector Dir;
		FVector InvDir;
		FVector Delta;
		CellIndex Step;
		static constexpr FVector cell_extent = SpatialGrid::CellExtent<Semantics>();

		int32 CalculateMaxSteps(const FVector& hit_point) const
		{
			const FVector delta = End - hit_point;
			
			return
			FMath::CeilToInt(FMath::Abs(delta.X) / Semantics::CellSize) + 
			FMath::CeilToInt(FMath::Abs(delta.Y) / Semantics::CellSize) +
			FMath::CeilToInt(FMath::Abs(delta.Z) / Semantics::CellSize) + 1;	
		}
		
		void Progress(CellIndex& current_cell, FVector& t_max) const
		{
			// Determine which axis is crossed next
			if (t_max.X < t_max.Y && t_max.X < t_max.Z)
			{
				current_cell.X += Step.X;
				t_max.X += Delta.X;
			}
			else if (t_max.Y < t_max.Z)
			{
				current_cell.Y += Step.Y;
				t_max.Y += Delta.Y;
			}
			else
			{
				current_cell.Z += Step.Z;
				t_max.Z += Delta.Z;
			}
		}
		
		template<typename F>
		void CheckAll(const Grid& grid, const CellIndex& offset, CellSet& checked_cells, F&& func) const
		{
			auto scan_element = [this, func = std::forward<F>(func)](const ElementId& id, const Element& element)
			{
				if (FVector hit_loc; element.Bounds.LineHitPoint(Start, End, Dir, InvDir, hit_loc))
				{
					func(id, element, hit_loc);
				}
			};
			
			auto scan_cell = [this, &grid, &scan_element](const Cell& cell)
			{
				if (cell.HasElements() && LineIntersectsBox(cell.GetBounds(), Start, InvDir))
				{
					cell.ForEachElement(grid, scan_element);
				}
			};
			
			// check (3x3x3) cube around current cell (including current cell)
			CellRange(1).ForEach([&](const CellIndex& index)
			{
				if(const CellIndex coords = index + offset; !checked_cells.contains(coords))
				{
					grid.GetCell(coords, scan_cell);
					checked_cells.insert(coords);
				}
			});
		}

		void CheckClosest(const Grid& grid, const CellIndex& offset, CellSet& checked_cells, QueryResult& closest) const
		{
			closest.Location = End;
			
			auto scan_element = [this, &closest](const ElementId id, const Element& element)
			{
				if (FVector hit_loc; element.Bounds.LineHitPoint(Start, End, Dir, InvDir, hit_loc))
				{
					if (!closest.BlockingHit || FVector::DistSquared(Start, hit_loc) < FVector::DistSquared(Start, closest.ImpactPoint))
					{
						closest.BlockingHit = true;
						closest.Location = closest.ImpactPoint = hit_loc;
						closest.ElementId = id;
					}
				}
			};
			
			auto scan_cell = [this, &grid, &scan_element](const Cell& cell)
			{
				if (cell.HasElements() && LineIntersectsBox(cell.GetBounds(), Start, InvDir))
				{
					cell.ForEachElement(grid, scan_element);
				}
			};
			
			// check (3x3x3) cube around current cell (including current cell)
			CellRange(1).ForEach(offset, [&](const CellIndex index)
			{
				if(const CellIndex coords = index + offset; !checked_cells.contains(coords))
				{
					grid.GetCell(coords, scan_cell);
					checked_cells.insert(coords);
				}
			});
		}
	};
}