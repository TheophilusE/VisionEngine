
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
#include <wrl.h> // For using Microsoft::WRL::ComPtr<T>

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

