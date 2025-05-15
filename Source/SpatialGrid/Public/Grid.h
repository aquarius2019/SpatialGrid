#pragma once

#include "SlotMap.h"
#include "SpatialGridUtils.h"
#include "unordered_dense.h"

namespace SpatialGrid
{
	template<typename Semantics>
	struct TSpatialGrid
	{
		static_assert(Semantics::CellSize > 0, "cell size must be greater than zero");
		static_assert(Semantics::MaxElementRadius < HalfCellSize<Semantics>(), "max element radius must be less than half cell size");

		using ElementData = typename Semantics::ElementData;
		
		struct Element
		{
			Element() = default;
			Element(const CellIndex& cell,const Bounds& bounds, ElementData&& data)
			: Cell(cell)
			, Bounds(bounds)
			, Data(std::move(data)) {}
			
			Element(const CellIndex& cell, const Bounds& bounds, const ElementData& data)
			: Cell(cell)
			, Bounds(bounds)
			, Data(data) {}
		
			CellIndex Cell = CellIndex(TNumericLimits<int32>::Max());
			Bounds Bounds;
			ElementData Data;
		};

		using ElementIds = ankerl::unordered_dense::set<ElementId>;
		
		struct Cell
		{
			Cell() = default;

			const FBox& GetBounds() const
			{
				return Bounds;
			}
			
			bool HasElements() const
			{
				return !Elements.empty();
			}

			template<typename F>
			void ForEachElement(const TSpatialGrid& grid, F&& func) const
			{
				for (const ElementId& id : Elements)
				{
					grid.Elements.ApplyAt(id, std::forward<F>(func));
				}
			}
			
		private:
			ElementIds Elements;
			FBox Bounds;
			friend struct TSpatialGrid;
		};

	private:
		using CellStorage = ankerl::unordered_dense::map<CellIndex, Cell>;

	public:
		TSpatialGrid() = default;
	
		explicit TSpatialGrid(const FVector& InOrigin) : Origin(InOrigin) {}

		double CellSize() const { return Semantics::CellSize; }

		int32 NumCells() const { return Cells.size(); }
	
		CellIndex LocationToCoordinates(const FVector& world_location) const
		{
			return RoundVecToInt((world_location - Origin) / Semantics::CellSize);
		}
	
		FVector CellCenter(const CellIndex& Coords) const
		{
			return FVector(
				Origin.X + (Coords.X * Semantics::CellSize),
				Origin.Y + (Coords.Y * Semantics::CellSize),
				Origin.Z + (Coords.Z * Semantics::CellSize));
		}
		
		ElementId AddElement(const Bounds& bounds, ElementData&& data)
		{
			checkf(bounds.GetRadius() < HalfCellSize<Semantics>(), TEXT("element radius must be less than cell extent"));
			
			const CellIndex coords = LocationToCoordinates(bounds.Origin);

			FScopeLock Lock(&CriticalSection);
			
			ElementId new_id = Elements.Insert(coords, bounds, std::move(data));
			Cell& cell = FindOrAddCell(coords);
			cell.Elements.insert(new_id);
			
			return new_id;
		}

		void RemoveElement(const ElementId id)
		{
			FScopeLock Lock(&CriticalSection);
			
			if (std::optional<Element> element = Elements.Remove(id))
			{
				if (auto it = Cells.find(element->Cell); it != Cells.end())
				{
					it->second.Elements.erase(id);
				}
			}
		}

		/** This function is not thread safe!!! */
		const Element* GetElement(const ElementId& id) const
		{
			return Elements.Get(id);
		}
		
		void ClearEmptyCells()
		{
			FScopeLock Lock(&CriticalSection);
			
			std::erase_if(Cells, [](const auto& entry)
			{
				const auto& [_, cell] = entry;
				return !cell.HasElements();
			});
		}

		void UpdateElementLocation(const ElementId id, const FVector& new_location)
		{
			Element* element = Elements.Get(id); if (!element) { return; }

			FScopeLock Lock(&CriticalSection);

			element->Bounds.Origin = new_location;
			
			const CellIndex new_coords = LocationToCoordinates(new_location);

			if (new_coords != element->Cell)
			{
				auto cell_it = Cells.find(element->Cell); check(cell_it != Cells.end());
				
				Cell& prev_cell = cell_it->second;
				prev_cell.Elements.erase(id);
				
				Cell& new_cell = FindOrAddCell(new_coords);
				new_cell.Elements.insert(id);
				element->Cell = new_coords;
			}
		}
		
		/// This function is not thread safe!!!
		FORCEINLINE const Cell* GetCell(const CellIndex& Coords) const
		{
			const auto it = Cells.find(Coords);
			return it != Cells.end() ? &(it->second) : nullptr;
		}

		/// This function is not thread safe!!!
		template<typename  F>
		void GetCell(const CellIndex& Coords, F&& func) const
		{
			if (const auto it = Cells.find(Coords); it != Cells.end())
			{
				func(it->second);
			}
		}

		template <typename IterFunc>
		void ForEachCell(IterFunc&& Func) const
		{
			for (const auto& [coords, cell] : Cells)
			{
				Func(coords, cell);
			}
		}

		template <typename IterFunc>
		void ForEachElement(IterFunc&& Func) const
		{
			for (const auto& [id, element] : Elements)
			{
				Func(id, element);
			}
		}

		bool IsCellWithinBounds(const CellIndex& Coords) const
		{
			return Bounds.IsInside(CellCenter(Coords));
		}

		const FVector& GetOrigin() const
		{
			return Origin;
		}
		
		const FBox& GetBounds() const
		{
			return Bounds;
		}

	private:
		FVector Origin = FVector::ZeroVector;
		TSlotMap<Element> Elements;
		CellStorage Cells;
		FBox Bounds;
		FCriticalSection CriticalSection;
		
		Cell& FindOrAddCell(const CellIndex& coords)
		{
			auto[it, is_new_cell] = Cells.try_emplace(coords);
			
			if (is_new_cell)
			{
				constexpr FVector cell_extent = SpatialGrid::CellExtent<Semantics>();
				const FVector cell_origin = CellCenter(coords);
				Bounds += FBox(cell_origin - cell_extent, cell_origin + cell_extent);
			}
			
			return it->second;
		}
	};
}