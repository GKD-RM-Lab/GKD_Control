//
// Created by rhine on 3/11/24.
//
#pragma once

#include "functional"
#include "map"

namespace IO {

    /** callback base define **/
    /**
     * Callback实现的基类，用于存储多种不同的Callback类型
     * @tparam Args 需要的类型
     */
    template<typename ...Args>
    class CallbackBase;

    template<typename Head, typename ...Args>
    class CallbackBase<Head, Args...> : public CallbackBase<Args...> {
       public:
        std::function<void(const Head &)> callback_fun;
    };

    template<typename Head>
    class CallbackBase<Head> {
       public:
        std::function<void(const Head &)> callback_fun;
    };

    /** callback element define **/
    /**
     * 辅助类，用于取出CallbackBase中，Idx类对应的父类
     * @tparam Idx 需要取出的父类所对应的类型
     * @tparam T 从这个CallbackBase取出
     */
    template<typename Idx, typename T>
    struct CallbackElement;

    template<typename Idx, typename Head, typename ...Tail>
    struct CallbackElement<Idx, CallbackBase<Head, Tail...>> : CallbackElement<Idx, CallbackBase<Tail...>> {
        using Inherited = CallbackElement<Idx, CallbackBase<Tail...>>;
        using ListType = typename Inherited::ListType;
    };

    template<typename Idx, typename ...Tail>
    struct CallbackElement<Idx, CallbackBase<Idx, Tail...>> {
        using ListType = CallbackBase<Idx, Tail...>;
    };

    /** callback define **/
    /**
     * 用于需要被HardwareManager管理的类继承，使其可以使用Callback，支持多种类型
     * @tparam Args 需要callback的类型
     */
    template<typename ...Args>
    class Callback;

    template<typename Head, typename Sec, typename ...Args>
    class Callback<Head, Sec, Args...> {
       public:
        template<typename T>
        void register_callback(const std::function<void(const T &)> &fun) {
            ((ListType <T> &) callback_set).callback_fun = fun;
        }

       protected:
        template<typename T>
        void callback(const T &val) {
            auto &fun = ((ListType <T> &) callback_set).callback_fun;
            if (fun) {
                fun(val);
            }
        }

       private:
        using BaseType = CallbackBase<Head, Sec, Args...>;
        BaseType callback_set;
        template<typename Idx>
        using ListType = typename CallbackElement<Idx, BaseType>::ListType;

    };

    template<typename Head>
    class Callback<Head> {
       public:
        template<class T>
        void register_callback(const std::function<void(const T &)> &fun) {
            callback_fun = fun;
        }

        void register_callback(const std::function<void(const Head &)> &fun) {
            callback_fun = fun;
        }

       protected:
        void callback(const Head &val) {
            if (callback_fun) {
                callback_fun(val);
            }
        }

       private:
        std::function<void(const Head &)> callback_fun;
    };

    /** callback key define**/
    template<typename Key, typename ...Args>
    class Callback_key {
       public:
        void callback_key(const Key &key, const Args &... args) {
            if (callback_map.count(key)) {
                callback_map[key](args...);
            }
        }

        void register_callback_key(const Key &key, const std::function<void(const Args &...)> &fun) {
            callback_map[key] = fun;
        }

       private:
        std::map<Key, std::function<void(const Args &...)>> callback_map;
    };

}  // namespace Hardware
