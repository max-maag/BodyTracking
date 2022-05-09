#pragma once

#include <memory>
#include <functional>
#include <type_traits>
#include <algorithm>
#include <iterator>

#include <Kore/Graphics4/Graphics.h>

#include "Utils.h"

namespace {
	// Is integral or float type
	template<typename T>
	concept arithmetic = std::is_arithmetic_v<std::remove_reference_t<T>>;

	// Is a Kore VertexBuffer or IndexBuffer
	template<typename B>
	concept BufferType = requires(B buffer) {
		{ buffer.lock() } -> std::contiguous_iterator;
		{ *(buffer.lock()) } -> arithmetic;
		buffer.unlock();
	};

	// Is a Kore VertexBuffer
	template<typename B>
	concept VertexBufferType = BufferType<B> && requires (B buffer) {
		{ buffer.stride() } -> std::convertible_to<size_t>;
	};
}

namespace utils {

	// Abstraction over Kore's VertexBuffer and IndexBuffer.
	template<BufferType Buffer>
	class GraphicsBuffer
	{
	public:
		// Primitive type of the buffer's content, as returned by Buffer::lock
		typedef std::remove_pointer_t<decltype(std::declval<Buffer>().lock())> PrimitiveType;

		// Function that takes a size and creates a unique_ptr to a Buffer with that size
		typedef std::function<std::unique_ptr<Buffer>(size_t)> BufferFactory;

		/* Ensures the buffer can hold at least size elements.
		 * 
		 * For IndexBuffers, an element is an index.
		 * For VertexBuffers, an element is a Vertex, not a single float!
		 */
		void ensureSize(size_t size);

	private:
		static const size_t DEFAULT_CAPACITY = 32;

		typedef GraphicsBuffer<Buffer> SelfType;

		BufferFactory createBuffer = [](auto size) { return nullptr; };

		std::unique_ptr<Buffer> buffer;


		// How many primitive elements are currently written to the buffer
		size_t primitiveSize;

		/* Appends count elements of it to the buffer.
		 * 
		 * Bounds are NOT checked, make sure that the data fits into the buffer.
		 * Type correctness is NOT checked, be sure to only append types that are layout compatible with PrimitiveType[].
		 */
		template<typename It>
			requires std::contiguous_iterator<It> and (sizeof(std::iter_value_t<It>) % sizeof(PrimitiveType) == 0)
		void append_n(It it, size_t count) {

			PrimitiveType* bufferEnd = &(buffer->lock()[primitiveSize]);

			std::copy_n(it, count, reinterpret_cast<std::iter_value_t<It>*>(bufferEnd));

			primitiveSize += count * sizeof(std::iter_value_t<It>) / sizeof(PrimitiveType);

			buffer->unlock();
		}

	public:
		GraphicsBuffer(BufferFactory createBuffer): createBuffer(createBuffer) {
			buffer = createBuffer(DEFAULT_CAPACITY);
			primitiveSize = 0;
		}

		template<typename = void> requires VertexBufferType<Buffer>
		size_t calculatePrimitiveCapacity() {
			return buffer.count() * buffer->stride() / sizeof(PrimitiveType);
		}

		template<typename = void> requires (not VertexBufferType<Buffer>)
			size_t calculatePrimitiveCapacity() {
			return buffer->count();
		}

		template<typename = void> requires VertexBufferType<Buffer>
		size_t calculateElementSize() {
			return primitiveSize / (buffer->stride() / sizeof(PrimitiveType));
		}

		template<typename = void> requires (not VertexBufferType<Buffer>)
			size_t calculateElementSize() {
			return primitiveSize;
		}

		/* Puts data for a new vertex into the buffer.
		 * t must contain all the data in the correct order as specified by the buffer's VertexStructure.
		 */
		template<typename T> requires VertexBufferType<Buffer>
		void put(T& t) {
			utils::runtimeAssert(sizeof(T) == buffer->stride(), "Size mismatch");
			ensureSize(calculateElementSize() + 1);
			append_n(&t, 1);
		}

		/* Puts data for a new vertex into the buffer.
		 * t must contain all the data in the correct order as specified by the buffer's VertexStructure.
		 */
		template<typename T> requires VertexBufferType<Buffer>
		void put(T&& t) {
			utils::runtimeAssert(sizeof(T) == buffer->stride(), "Size mismatch");
			ensureSize(calculateElementSize() + 1);
			append_n(&t, 1);
		}

		// Puts a new index into an IndexBuffer.
		template<typename = void> requires (not VertexBufferType<Buffer>)
		void put(PrimitiveType value) {
			ensureSize(calculateElementSize() + 1);
			append_n(&value, 1);
		}
		
		/* Puts data for one or more vertices into the buffer.
		 * Each element must contain all the data in the correct order as specified by the buffer's VertexStructure.
		 */
		template<typename It> requires std::contiguous_iterator<It> and VertexBufferType<Buffer>
		void putMany(It begin, It end) {
			utils::runtimeAssert(sizeof(std::iter_value_t<It>) == buffer->stride(), "Size mismatch");

			size_t sizeDataNew = std::distance(begin, end);
			ensureSize(calculateElementSize() + sizeDataNew);
			append_n(begin, sizeDataNew);
		}

		// Puts multiple indices into an IndexBuffer.
		template<typename It> requires
			std::contiguous_iterator<It> and
			std::same_as<std::iter_value_t<It>, PrimitiveType> and
			(not VertexBufferType<Buffer>)
		void putMany(It begin, It end) {
			size_t sizeDataNew = std::distance(begin, end);
			ensureSize(calculateElementSize() + sizeDataNew);
			append_n(begin, sizeDataNew);
		}

		// Resets the write position to the beginning of the buffer.
		void clear() {
			primitiveSize = 0;
		}

		Buffer* getBuffer() {
			return &*buffer;
		}
	};

	template<BufferType Buffer>
	void GraphicsBuffer<Buffer>::ensureSize(size_t size) {
		if (size <= buffer->count()) {
			return;
		}

		auto bufferNew = createBuffer(nextOrEqualPowerOfTwo(size));

		PrimitiveType* dataSrc = buffer->lock();
		PrimitiveType* dataDst = bufferNew->lock();

		std::copy_n(dataSrc, primitiveSize, dataDst);

		buffer->unlock();
		bufferNew->unlock();

		buffer = std::move(bufferNew);
	}

	typedef GraphicsBuffer<Kore::Graphics4::VertexBuffer> G4VertexBuffer;
	typedef GraphicsBuffer<Kore::Graphics4::IndexBuffer> G4IndexBuffer;
}