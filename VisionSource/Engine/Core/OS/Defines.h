#pragma once

/// Future data type for reading future variables
#include <future>
template <class T>
using Future = std::future<T>;

/// Promise data types for sharing futures
#include <future>
template <class T>
using Promise = std::promise<T>;

/// Mutex for mutual exclusion
#include <mutex>
typedef std::mutex Mutex;
typedef std::recursive_mutex RecursiveMutex;

/// Atomic data type
#include <atomic>
template <class T>
using Atomic = std::atomic<T>;

// Smart pointers
#include <memory>
/// std::unique_ptr
template <class T>
using Ptr = std::unique_ptr<T>;
/// std::shared_ptr
template <class T>
using Ref = std::shared_ptr<T>;
/// std::weak_ptr
template <class T>
using Weak = std::weak_ptr<T>;
//#include <wrl.h> // For using Microsoft::WRL::ComPtr<T>

// Serialization streams
#include <fstream>
/// std::fstream
typedef std::fstream InputOutputFileStream;
/// std::ofstream
typedef std::ofstream OutputFileStream;
/// std::ifstream
typedef std::ifstream InputFileStream;
#include <sstream>
/// std::stringstream
typedef std::stringstream StringStream;

// Containers
#include <string>
/// std::string
typedef std::string String;

#include <map>
/// std::map
template <class P, class Q>
using Map = std::map<P, Q>;

#include <unordered_map>
/// std::unordered_map
template <class P, class Q>
using HashMap = std::unordered_map<P, Q>;

#include <utility>
/// std::tuple
template <typename... P>
using Tuple = std::tuple<P...>;
/// std::pair
template <class P, class Q>
using Pair = std::pair<P, Q>;

#include <optional>
/// std::optional
template <class T>
using Optional = std::optional<T>;

#include <vector>
/// std::vector
template <class T>
using Vector = std::vector<T>;

Vector<String> Split(const String& s, char delim);

#include <array>
/// std::array
template <class T, int N>
using Array = std::array<T, N>;

#include <stack>
/// std::stack
template <class T>
using Stack = std::stack<T>;

#include <filesystem>
/// std::filesystem::path
using FilePath = std::filesystem::path;

#include <DirectXColors.h>
/// DirectX::Colors
namespace ColorPresets = DirectX::Colors;

#include <functional>
/// std::function
template <class T>
using Function = std::function<T>;

#include <variant>
#include "BasicMath.hpp"
/// Vector of std::variant of bool, int, char, float, String, Vector2, Vector3, Vector4, Matrix
typedef Vector<std::variant<bool, int, char, float, String, Diligent::float2, Diligent::float3, Diligent::float4, Diligent::float2x2, Diligent::float3x3, Diligent::float4x4>> VariantVector;
/// A variant able to hold multiple kinds of data, one at a time.
using Variant = std::variant<bool, int, char, float, String, Vector<String>, Diligent::float2, Diligent::float3, Diligent::float4, Diligent::float2x2, Diligent::float3x3, Diligent::float4x4, VariantVector>;

/// Extract the value of type TypeName from a Variant
template <typename P, typename Q>
P Extract(const Q& v)
{
	return std::get<P>(v);
}

#include <nlohmann/json.hpp>
/// Namespace for the JSON library
namespace JSON = nlohmann;

namespace nlohmann
{
template <>
struct adl_serializer<Diligent::float2>
{
	static void to_json(json& j, const Diligent::float2& v)
	{
		j["x"] = v.x;
		j["y"] = v.y;
	}

	static void from_json(const json& j, Diligent::float2& v)
	{
		v.x = j.at("x");
		v.y = j.at("y");
	}
};
template <>
struct adl_serializer<Diligent::float3>
{
	static void to_json(json& j, const Diligent::float3& v)
	{
		j["x"] = v.x;
		j["y"] = v.y;
		j["z"] = v.z;
	}

	static void from_json(const json& j, Diligent::float3& v)
	{
		v.x = j.at("x");
		v.y = j.at("y");
		v.z = j.at("z");
	}
};
template <>
struct adl_serializer<Diligent::float4>
{
	static void to_json(json& j, const Diligent::float4& v)
	{
		j["x"] = v.x;
		j["y"] = v.y;
		j["z"] = v.z;
		j["w"] = v.w;
	}

	static void from_json(const json& j, Diligent::float4& v)
	{
		v.x = j.at("x");
		v.y = j.at("y");
		v.z = j.at("z");
		v.w = j.at("w");
	}
};
/*
template <>
struct adl_serializer<Color>
{
	static void to_json(json& j, const Color& v)
	{
		j["r"] = v.R();
		j["g"] = v.G();
		j["b"] = v.B();
		j["a"] = v.A();
	}

	static void from_json(const json& j, Color& v)
	{
		v.x = j.at("r");
		v.y = j.at("g");
		v.z = j.at("b");
		v.w = j.at("a");
	}
};
*/
template <>
struct adl_serializer<Diligent::Quaternion>
{
	static void to_json(json& j, const Diligent::Quaternion& v)
	{
		j["x"] = v.q.x;
		j["y"] = v.q.y;
		j["z"] = v.q.z;
		j["w"] = v.q.w;
	}

	static void from_json(const json& j, Diligent::Quaternion& v)
	{
		v.q.x = j.value("x", 0.0f);
		v.q.y = j.value("y", 0.0f);
		v.q.z = j.value("z", 0.0f);
		v.q.w = j.value("w", 0.0f);
	}
};
template <>
struct adl_serializer<Diligent::float4x4>
{
	static void to_json(json& j, const Diligent::float4x4& v)
	{
		for (int x = 0; x < 4; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				j.push_back(v.m[x][y]);
			}
		}
	}

	static void from_json(const json& j, Diligent::float4x4& v)
	{
		for (int x = 0; x < 4; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				v.m[x][y] = j[x * 4 + y];
			}
		}
	}
};
/*
template <>
struct adl_serializer<BoundingBox>
{
	static void to_json(json& j, const BoundingBox& v)
	{
		j["center"] = (float3)v.Center;
		j["extents"] = (float3)v.Extents;
	}

	static void from_json(const json& j, BoundingBox& v)
	{
		v.Center = (float3)j.value("center", Vector3::Zero);
		v.Extents = (float3)j.value("extents", Vector3 { 0.5f, 0.5f, 0.5f });
	}
};
*/
}
