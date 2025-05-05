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

	private:
		struct Element
		{
			Element() = default;
			Element(const CellIndex& InCoord, ElementData&& InData) : Cell(InCoord), Data(std::move(InData)) {}
			Element(const CellIndex& InCoord, const ElementData& InData): Cell(InCoord), Data(InData) {}
		
			CellIndex Cell = CellIndex(TNumericLimits<int32>::Max());
			ElementData Data;
		};

		using ElementIds = ankerl::unordered_dense::set<ElementId>;
		
	public:
		struct Cell
		{
			Cell() = default;
			
			bool HasElements() const { return Elements.empty() == false; }

			template<typename F>
			void ForEachElement(const TSpatialGrid& Grid, F&& Func) const
			{
				for (const ElementId& Id : Elements)
				{
					Grid.Elements.ApplyAt(Id, std::forward<F>(Func));
				}
			}
		
		private:
			ElementIds Elements;
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

		template<typename ...Args>
		ElementId AddElement(Args&&... args)
		{
			ElementData data(std::forward<Args>(args)...);
			
			const FBoxSphereBounds& ElementBounds = Semantics::ElementBounds(data);

			checkf(ElementBounds.SphereRadius < HalfCellSize<Semantics>(),
				TEXT("element radius must be less than cell extent"));
			
			const CellIndex coords = LocationToCoordinates(ElementBounds.Origin);

			FScopeLock Lock(&CriticalSection);
				
			ElementId new_id = Elements.Insert(coords, std::move(data));
			Cell& cell = FindOrAddCell(coords);
			cell.Elements.insert(new_id);
			
			return new_id;
		}

		void RemoveElement(const ElementId id)
		{
			FScopeLock Lock(&CriticalSection);
			
			if (std::optional<Element> Handle = Elements.Remove(id))
			{
				if (auto it = Cells.find(Handle->Cell); it != Cells.end())
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
			
			std::erase_if(Cells, [](const std::pair<CellIndex, Cell>& entry)
			{
				const auto& [_, cell] = entry;
				return !cell.HasElements();
			});
		}

		void UpdateElementLocation(const ElementId id, const FVector& new_location)
		{
			FScopeLock Lock(&CriticalSection);

			Element* element = Elements.Get(id); if (!element) { return; }
			
			Semantics::SetElementLocation(element->Data, new_location);
			const CellIndex new_coords = LocationToCoordinates(new_location);

			if (new_coords != element->Cell)
			{
				auto cell_it = Cells.find(element->Cell); if (cell_it == Cells.end()) { return; }
				Cell& prev_cell = cell_it->second;
				prev_cell.Elements.erase(id);
				
				Cell& new_cell = FindOrAddCell(new_coords);
				new_cell.Elements.insert(id);
				element->Cell = new_coords;
			}
		}
		
		FORCEINLINE const Cell* GetCell(const CellIndex& Coords) const
		{
			const auto it = Cells.find(Coords);
			return it != Cells.end() ? &(it->second) : nullptr;
		}

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
			for (const auto& [Coords, Cell] : Cells)
			{
				Func(Coords, Cell);
			}
		}

		template <typename IterFunc>
		void ForEachElement(IterFunc&& Func) const
		{
			for (const auto& [id, Handle] : Elements)
			{
				Func(id, Handle.Element);
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
		CellStorage Cells;
		TSlotMap<Element> Elements;
		FBox Bounds;
		FCriticalSection CriticalSection;
		
		Cell& FindOrAddCell(const CellIndex& coords)
		{
			auto[it, is_new_cell] = Cells.try_emplace(coords);
			
			if (is_new_cell)
			{
				const FVector cell_origin = CellCenter(coords);
				constexpr FVector cell_extent = SpatialGrid::CellExtent<Semantics>();
				Bounds += FBox(cell_origin - cell_extent, cell_origin + cell_extent);
			}
			
			return it->second;
		}
	};
}