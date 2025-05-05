#pragma once
#include "SpatialGrid.h"
#include "SpatialGridTypes.h"
#include "Logging/StructuredLog.h"

namespace SpatialGrid
{
	struct Slot
	{
		/// Even = vacant, odd = occupied.
		uint32_t Version;
		/// Index to dense array when occupied, next free slot otherwise.
		uint32_t IdxOrFree;

		FORCEINLINE bool IsOccupied() const
		{
			return (Version % 2) != 0;
		}
	};
	
	template <typename V>
	struct TSlotMap
	{
		TSlotMap() {}
		
		explicit TSlotMap(size_t Capacity)
		{
			// Slots is resized as needed.
			Dense.reserve(Capacity);
		}

		template<typename ...Args>
		ElementId Insert(Args&&... args)
		{
			if (Dense.size() >= UINT32_MAX)
			{
				UE_LOGFMT(LogSpatialGrid, Fatal, "SparseSet number of elements overflow");
				return ElementId();
			}

			size_t index = FreeHead;
			uint32_t version;

			if (index < Dense.size())
			{
				Slot& slot = Slots[index];
				version = slot.Version | 1;
				FreeHead = slot.IdxOrFree;

				slot.Version = version;
				slot.IdxOrFree = Dense.size();
			}
			else
			{
				version = 1;
				Slots.push_back(Slot{1, Dense.size()});
				FreeHead = Slots.size();
			}
			
			ElementId id = ElementId(index, version);
			Dense.push_back(std::make_pair(id, std::forward<Args>(args)...));

			return id;
		}
		
		std::optional<V> Remove(const ElementId& id) 
		{
			if (id.Index >= Slots.size()) [[unlikely]]
			{
				return std::nullopt;
			}

			Slot& slot = Slots[id.Index];
			
			if (!slot.IsOccupied() || slot.Version != id.Version)
			{
				return std::nullopt;
			}

			const size_t dense_idx = slot.IdxOrFree;
			
			check(dense_idx < Dense.size());
			check(Dense[dense_idx].first.Index == id.Index);

			V value = std::move(Dense[dense_idx].second);

			// Free slot.
			slot.Version += 1;
			slot.IdxOrFree = FreeHead;
			FreeHead = id.Index;
			
			if (dense_idx != (Dense.size() - 1))
			{
				Dense[dense_idx] = std::move(Dense.back());
				Slots[Dense[dense_idx].first.Index].IdxOrFree = dense_idx;
			}
			
			Dense.pop_back();
			return value;
		}

		bool Contains(const ElementId& Id) const {
			if (Id.Index >= Slots.size())
			{
				return false;
			}

			const Slot& Slot = Slots[Id.Index];
			return Slot.IsOccupied() && Slot.Version != Id.Version;
		}

		const V* Get(const ElementId& id) const
		{
			if (id.Index >= Slots.size()) [[unlikely]]
			{
				return nullptr;
			}

			const Slot& slot = Slots[id.Index];
			
			return (slot.IsOccupied() && slot.Version == id.Version)
				? &Dense[slot.IdxOrFree].second
				: nullptr;
		}

		V* Get(const ElementId& id)
		{
			if (id.Index >= Slots.size()) [[unlikely]]
			{
				return nullptr;
			}

			Slot& slot = Slots[id.Index];
			
			return (slot.IsOccupied() && slot.Version == id.Version)
				? &Dense[slot.IdxOrFree].second
				: nullptr;
		}

		template<typename F>
		void ApplyAt(const ElementId& id, F&& Func) const
		{
			if (id.Index < Slots.size()) [[unlikely]]
			{
				return;
			}

			if (const Slot& slot = Slots[id.Index]; slot.IsOccupied() && slot.Version == id.Version) [[likely]]
			{
				check(slot.IdxOrFree < Dense.size());
				const auto&[id, value] = Dense[slot.IdxOrFree];
				Func(id, value);
			}
		}
		
	private:
		std::vector<std::pair<ElementId, V>> Dense = {};
		std::vector<Slot> Slots = {};
		size_t FreeHead = 0;

		// Iterators
	public:
		using iterator = typename std::vector<std::pair<ElementId, V>>::iterator;
		using const_iterator = typename std::vector<std::pair<ElementId, V>>::const_iterator;
		using reverse_iterator = typename std::vector<std::pair<ElementId, V>>::reverse_iterator;
		using const_reverse_iterator = typename std::vector<std::pair<ElementId, V>>::const_reverse_iterator;
		
		iterator begin() noexcept { return Dense.begin(); }
		iterator end() noexcept { return Dense.end(); }
		const_iterator begin() const noexcept { return Dense.begin(); }
		const_iterator end() const noexcept { return Dense.end(); }
		const_iterator cbegin() const noexcept { return Dense.cbegin(); }
		const_iterator cend() const noexcept { return Dense.cend(); }
		reverse_iterator rbegin() noexcept { return Dense.rbegin(); }
		reverse_iterator rend() noexcept { return Dense.rend(); }
		const_reverse_iterator rbegin() const noexcept { return Dense.rbegin(); }
		const_reverse_iterator rend() const noexcept { return Dense.rend(); }
		const_reverse_iterator crbegin() const noexcept { return Dense.crbegin(); }
		const_reverse_iterator crend() const noexcept { return Dense.crend(); }
	};
}
