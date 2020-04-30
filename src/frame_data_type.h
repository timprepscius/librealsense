#pragma once

#include <cstddef>
#include <thread>
#include <mutex>
#include <map>
#include <vector>
#include "types.h"

template<typename T>
class static_framedata_allocator_array
{
public:
    typedef std::mutex Mutex;
    typedef std::lock_guard<Mutex> Lock;

    typedef static_framedata_allocator_array<T> self;

protected:
    static self *instance;

    Mutex mutex;
    struct Deallocation {
        T *t;
        size_t n;
    } ;
    std::vector<Deallocation> deallocations;


public:
    static_framedata_allocator_array() 
    {
        deallocations.reserve(128);
    }

    virtual ~static_framedata_allocator_array() 
    {
        for (auto &a : deallocations)
            delete a.t;
    }

    static self *get_instance () 
    {
        if (instance == nullptr)
            instance = new self();

        return instance;
    }


    static_framedata_allocator_array(const self&) = delete;
    self& operator=(const self&) = delete;

    T* allocate(std::size_t n)
    {
        {
            Lock l(mutex);

            // favor the most recently deallocated
            for (auto di=deallocations.rbegin(); di!=deallocations.rend(); ++di)
            {
                auto &d = *di;
                if (d.n == n)
                {
                    auto *t = d.t;

                    if (di != deallocations.rbegin())
                        d = deallocations.back();

                    deallocations.pop_back();

                    return t;
                }
            }
        }

        // std::cout << "static_framedata_allocator_array::allocate new " << n << std::endl;
        return static_cast<T*>(::operator new(n * sizeof(T)));
    }

    void deallocate(T* a, std::size_t n)
    {
        Lock l(mutex);
        deallocations.push_back({a, n});
    }
};

template<typename T>
static_framedata_allocator_array<T> *static_framedata_allocator_array<T>::instance = nullptr;



template<typename T>
class static_framedata_allocator_map
{
public:
    typedef std::mutex Mutex;
    typedef std::lock_guard<Mutex> Lock;
    typedef static_framedata_allocator_map<T> self;

protected:
    static self *instance;

    Mutex mutex;
    std::map<std::size_t, std::vector<T*>> deallocations;


public:
    static_framedata_allocator_map() 
    {
    }

    virtual ~static_framedata_allocator_map() 
    {
        for (auto &av : deallocations)
            for (auto *a : av.second)
                delete a;
    }

    static self *get_instance () 
    {
        if (instance == nullptr)
            instance = new self();

        return instance;
    }


    static_framedata_allocator_map(const self&) = delete;
    self& operator=(const self&) = delete;

    T* allocate(std::size_t n)
    {
        {
            Lock l(mutex);
            auto &av = deallocations[n];
            if (!av.empty())
            {
                auto *a = av.back();
                av.pop_back();

                return a;
            }
        }

        // std::cout << "static_framedata_allocator::allocate new " << n << std::endl;
        return static_cast<T*>(::operator new(n * sizeof(T)));
    }

    void deallocate(T* a, std::size_t n)
    {
        Lock l(mutex);
        deallocations[n].push_back(a);
    }
};

template<typename T>
static_framedata_allocator_map<T> *static_framedata_allocator_map<T>::instance = nullptr;


template<typename T>
using static_framedata_allocator = static_framedata_allocator_array<T>;

template<typename T>
class framedata_allocator
{
public:
    typedef T value_type;
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    typedef std::true_type is_always_equal;

    // rebind allocator to type U
    template <class U>
    struct rebind {
    typedef framedata_allocator<U> other;
    };

public:
    framedata_allocator() {}

    T* allocate(std::size_t n)
    {
        return static_framedata_allocator<T>::get_instance()->allocate(n);
    }

    void deallocate(T* a, std::size_t n)
    {
        return static_framedata_allocator<T>::get_instance()->deallocate(a, n);
    }

    bool operator ==(const framedata_allocator<T> &rhs) const
    {
        return true;
    }

    bool operator !=(const framedata_allocator<T> &rhs) const
    {
        return false;
    }

    void destroy (T *p) 
    { 
    }
};

template<typename T>
class vector_intrinsic
{
protected:
    T *data_;
    std::size_t size_;

    void clear()
    {
        data_ = nullptr;
        size_ = 0;
    }

    void allocate (std::size_t size__)
    {
        deallocate();

        size_ = size__;
        data_ = static_framedata_allocator<T>::get_instance()->allocate(size_);
    }

    void deallocate ()
    {
        if (data_ != nullptr)
        {
            static_framedata_allocator<T>::get_instance()->deallocate(data_, size_);
            clear();
        }
    }

    void move_from(vector_intrinsic &&m)
    {
        data_ = m.data_;
        size_ = m.size_;
        m.clear();
    }

public:
    vector_intrinsic()
    {
        clear();
    }

    vector_intrinsic(const vector_intrinsic &) = delete;
    vector_intrinsic &operator=(const vector_intrinsic &) = delete;

    ~vector_intrinsic()
    {
        deallocate();
    }

    vector_intrinsic(vector_intrinsic &&m)
    {
        move_from(std::move(m));
    }

    vector_intrinsic &operator =(vector_intrinsic &&m)
    {
        deallocate();
        move_from(std::move(m));
        return *this;
    }

    template<typename V>
    void assign(V begin_, V end_)
    {
        allocate(std::distance(begin_, end_));
        std::copy(begin_, end_, data_);
    }

    void assign(const T *begin_, const T *end_)
    {
        allocate(std::distance(begin_, end_));
        std::memcpy((void *)data_, (void *)begin_, size_ * sizeof(T));
    }

    const T *begin() const
    {
        return data_;
    }

    T *begin()
    {
        return data_;
    }

    const T *end() const
    {
        return data_ + size_;
    }

    T *end()
    {
        return data_ + size_;
    }

    const T *data() const
    {
        return data_;
    }

    T *data()
    {
        return data_;
    }


    std::size_t size () const
    {
        return size_;
    }

    void resize_with_initialization(size_t n, T value)
    {
        allocate(n);

        auto begin_ = begin();
        auto end_ = end();
        for (auto *it = begin_; it != end_; ++it)
            *it = value;
    }

    void resize_without_initialization(size_t n, T value)
    {
        allocate(n);
    }

    void resize(size_t n, T value)
    {
        // why is this being called at all?
        if (n < 1024)
            resize_with_initialization(n, value);
        else
           resize_without_initialization(n, value);
    }

} ;

typedef vector_intrinsic<byte> frame_data_type;
// typedef std::vector<byte, framedata_allocator<byte>>