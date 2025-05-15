#pragma once

#include <optional>

#include "Grid.h"
#include "SpatialGridQueryResult.h"
#include "SpatialGridUtils.h"

namespace SpatialGrid
{
	template<typename GridSemantics>
	struct TLineTrace
	{
		using Grid    = TSpatialGrid<GridSemantics>;
		using Cell    = typename Grid::Cell;
		using Element = typename Grid::Element;
		using CellSet = ankerl::unordered_dense::set<CellIndex>;
		
		TLineTrace(const FVector& start, const FVector& end_)
		: m_start(start)
		, m_end(end_)
		, m_dir((end_ - start).GetSafeNormal())
		, m_inv_dir(m_dir.Reciprocal())
		, m_delta(
			FMath::Abs(GridSemantics::CellSize * m_inv_dir.X),
			FMath::Abs(GridSemantics::CellSize * m_inv_dir.Y),
			FMath::Abs(GridSemantics::CellSize * m_inv_dir.Z))
		, m_step(m_dir.X > 0 ? 1 : -1, m_dir.Y > 0 ? 1 : -1, m_dir.Z > 0 ? 1 : -1) {}
		
		TLineTrace(const FVector& start, const FVector& direction, const double length)
		: TLineTrace(start, start + (direction * length)) {}
		
		template<typename IterFunc>
		void Multi(const TSpatialGrid<GridSemantics>& grid, IterFunc&& func) const
		{
			// check that line intersects current grid bounds
			FVector hit_point;
			if (!LineBoxHitPoint(grid.GetBounds(), m_start, m_end, m_dir, m_inv_dir, hit_point))
			{
				return;
			}

			// TODO: figure out a good algorithm for reserving memory
			CellSet   checked_cells(100);
			CellIndex current_cell = grid.LocationToCoordinates(hit_point);
			
			const FVector start_cell_origin = grid.CellCenter(current_cell);
			const FVector t1 = ((start_cell_origin - cell_extent) - hit_point) * m_inv_dir;
			const FVector t2 = ((start_cell_origin + cell_extent) - hit_point) * m_inv_dir;
			const CellIndex end_cell = grid.LocationToCoordinates(m_end);

			FVector t_max = FVector::Max(t1, t2);

			if (hit_point != m_start)
			{
				progress(current_cell, t_max);
			}
			
			while(true)
			{
				check_cells(grid, current_cell, checked_cells, std::forward<IterFunc>(func));

				if (current_cell == end_cell || !grid.IsCellWithinBounds(current_cell))
				{
					break;
				}

				progress(current_cell, t_max);
			}
		}
		
		QueryResult Single(const Grid& grid) const
		{
			QueryResult result = {};
			
			// check that line intersects current grid bounds
			FVector hit_point;
			if (LineBoxHitPoint(grid.GetBounds(), m_start, m_end, m_dir, m_inv_dir, hit_point) == false)
			{
				return result;
			}

			// TODO: figure out a good algorithm for reserving memory
			CellSet   checked_cells(100);
			CellIndex current_cell = grid.LocationToCoordinates(hit_point);
			
			const FVector start_cell_origin = grid.CellCenter(current_cell);
			const FVector t1 = ((start_cell_origin - cell_extent) - hit_point) * m_inv_dir;
			const FVector t2 = ((start_cell_origin + cell_extent) - hit_point) * m_inv_dir;
			const CellIndex end_cell = grid.LocationToCoordinates(m_end);

			FVector t_max = FVector::Max(t1, t2);

			if (hit_point != m_start)
			{
				progress(current_cell, t_max);
			}
			
			// Maybe use a for loop to code more defensively
			// Should be able to estimate the number of cells to check from the length of the trace
			while(true) 
			{
				check_closest(grid, current_cell, checked_cells, result);

				if (result.BlockingHit || current_cell == end_cell || !grid.IsCellWithinBounds(current_cell))
				{
					break;
				}

				progress(current_cell, t_max);
			}
	
			return result;
		}
		
	private:
		FVector m_start;
		FVector m_end;
		FVector m_dir;
		FVector m_inv_dir;
		FVector m_delta;
		CellIndex m_step;
		static constexpr FVector cell_extent = SpatialGrid::CellExtent<GridSemantics>();

		void progress(CellIndex& current_cell, FVector& t_max) const
		{
			// Determine which axis is crossed next
			if (t_max.X < t_max.Y && t_max.X < t_max.Z)
			{
				current_cell.X += m_step.X;
				t_max.X += m_delta.X;
			}
			else if (t_max.Y < t_max.Z)
			{
				current_cell.Y += m_step.Y;
				t_max.Y += m_delta.Y;
			}
			else
			{
				current_cell.Z += m_step.Z;
				t_max.Z += m_delta.Z;
			}
		}
		
		template<typename F>
		void check_cells(const Grid& grid, const CellIndex& offset, CellSet& checked_cells, F&& func) const
		{
			// check (3x3x3) cube around current cell (including current cell)
			CellRange(1).ForEach([&](const CellIndex& index)
			{
				const CellIndex coords = index + offset;
				
				if(checked_cells.contains(coords)) { return; }
				
				grid.GetCell(coords, [this, grid, &func](const Cell& cell)
				{
					if (cell.HasElements() && LineIntersectsBox(cell.GetBounds(), m_start, m_inv_dir))
					{
						cell.ForEachElement(grid, [&](auto&, const Element& element)
						{
							if (std::optional<FVector> hit_loc =
							GridSemantics::LineHitPoint(element, m_start, m_end, m_dir, m_inv_dir))
							{
								func(element, *hit_loc);
							}
						});
					}
				});
				
				checked_cells.insert(coords);
			});
		}

		void check_closest(const Grid& grid, const CellIndex& offset, CellSet& checked_cells, QueryResult& closest) const
		{
			using GridCell = typename TSpatialGrid<GridSemantics>::Cell;
			using ElementType = typename GridSemantics::ElementData;
			
			// check (3x3x3) cube around current cell (including current cell)
			CellRange(1).ForEach(offset, [&](CellIndex coords)
			{
				if(checked_cells.contains(coords)) { return; }
				
				const GridCell* cell = grid.GetCell(coords);
				
				if (cell && cell->HasElements() && LineIntersectsBox(cell->GetBounds(), m_start, m_inv_dir))
				{
					cell->ForEachElement(grid, [&](const ElementId id, const ElementType& element)
					{
						if (const std::optional<FVector> hit_loc =
							GridSemantics::LineHitPoint(element, m_start, m_end, m_dir, m_inv_dir))
						{
							if (closest.BlockingHit == false ||
								FVector::DistSquared(m_start, *hit_loc) < FVector::DistSquared(m_start, closest.ImpactPoint))
							{
								closest.BlockingHit = true;
								closest.Location = closest.ImpactPoint = *hit_loc;
								closest.ElementId = id;
							}
						}
					});
				}

				checked_cells.insert(coords);
			});

			return closest;
		}
	};
}
