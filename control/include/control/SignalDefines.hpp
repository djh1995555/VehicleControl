//
// Created by yarten on 2020/3/25.
//

#pragma once

#include <nox_msgs.h>

/// interfaces
namespace defines
{
    // 该宏用以简化与Type一一对应的代码
#define SIGNAL_BOOL_TYPE_INSTANTIATION(F)       \
    F(Horn,               HORN)                 \
    F(HighBeam,           HIGH_BEAM)            \
    F(LowBeam,            LOW_BEAM)             \
    F(RightTurnIndicator, RIGHT_TURN_INDICATOR) \
    F(LeftTurnIndicator,  LEFT_TURN_INDICATOR)  \
    F(MarkerLight,        MARKER_LIGHT)         \
    F(HazardLight,        HAZARD_LIGHT)         \
    F(FogLight,           FOG_LIGHT)            \
    F(Handbrake,          HANDBRAKE)            \
    F(Engine,             ENGINE)               \
    F(Automatic,          AUTOMATIC)            \
    F(Emergency,          EMERGENCY)            \
    F(FullLoaded,         FULL_LOADED)

#define SIGNAL_VALUE_TYPE_INSTANTIATION(F)      \
    F(Steer,              STEER)                \
    F(Throttle,           THROTTLE)             \
    F(Brake,              BRAKE)                \
    F(Wheel,              WHEEL)                \
    F(Speed,              SPEED)                \
    F(BucketAngle,        BUCKET_ANGLE)         \
    F(EngineSpeed,        ENGINE_SPEED)         \
    F(EngineTorque,       ENGINE_TORQUE)

#define SIGNAL_ENUM_TYPE_INSTANTIATION(F)       \
    F(Bucket,             BUCKET)               \
    F(Gear,               GEAR)                 \
    F(BucketPosition,     BUCKET_POSITION)

    enum BucketState
    {
        HOLD, LIFT, DROP
    };

    enum GearState
    {
        P = -2, R = -1, N = 0, G1, G2, G3, G4, G5, G6, D
    };

    enum BucketPositionState
    {
        BOTTOM, MIDDLE, TOP, BLOCKED
    };

    /**
     * @brief 定义了信号的全部类型（具体到对应控制对象）
     */
    enum Type
    {
#define TypeDecl(Name, Type) Type,
        UNDEFINED = 0,
        SIGNAL_BOOL_TYPE_INSTANTIATION(TypeDecl)
        SIGNAL_VALUE_TYPE_INSTANTIATION(TypeDecl)
        SIGNAL_ENUM_TYPE_INSTANTIATION(TypeDecl)
#undef TypeDecl
    };

    template <class T>
    class Value
    {
    public:
        explicit Value(Type type = UNDEFINED);

        inline void Enable();

        inline void Disable();

        inline void Set(T value);

        inline void FromMsg(const nox_msgs::Signal & msg);

    public:
        [[nodiscard]] inline bool IsEnable() const;

        [[nodiscard]] inline bool IsDisable() const;

        [[nodiscard]] inline T Get() const;

        [[nodiscard]] inline Type GetType() const;

        [[nodiscard]] inline nox_msgs::Signal ToMsg() const;

    private:
        Type _type = UNDEFINED;
        T    _value = T();
        bool _enable = false;
    };

    /**
     * @brief 集成了全部已知的状态量，包括Button、Slider、Switch类型的接口
     * 1. Button: bool 类型
     * 2. Slider: double 类型
     * 3. Switch: 枚举类型（具体为其[名字+State]，如名为XXX的对象类型为 XXXState）
     */
    struct Panel
    {
#define ButtonDecl(Name, Type) Value<bool> Name{Type};
        SIGNAL_BOOL_TYPE_INSTANTIATION(ButtonDecl);
#undef ButtonDecl
#define SliderDecl(Name, Type) Value<double> Name{Type};
        SIGNAL_VALUE_TYPE_INSTANTIATION(SliderDecl);
#undef SliderDecl
#define SwitchDecl(Name, Type) Value<Name##State> Name{Type};
        SIGNAL_ENUM_TYPE_INSTANTIATION(SwitchDecl)
#undef SwitchDecl

        [[nodiscard]] inline nox_msgs::SignalArray ToMsgs() const;

        inline void FromMsgs(const nox_msgs::SignalArray & msgs);

        inline void Reset();

        inline void Enable();

        inline void Disable();
    };

    inline std::string ToString(Type type);
    inline std::string ToString(bool value);
    inline std::string ToString(double value);
    inline std::string ToString(GearState value);
    inline std::string ToString(BucketState value);
    inline std::string ToString(BucketPositionState value);
}

/// implementation
namespace defines
{
    template<class T>
    Value<T>::Value(Type type)
        : _type(type)
    {}

    template<class T>
    void Value<T>::Enable()
    {
        _enable = true;
    }

    template<class T>
    void Value<T>::Disable()
    {
        _enable = false;
    }

    template<class T>
    void Value<T>::Set(T value)
    {
        _value = std::move(value);
    }

    template<class T>
    void Value<T>::FromMsg(const nox_msgs::Signal &msg)
    {
        if(msg.type == _type)
        {
            _enable = msg.code != 0;

            if(msg.data.size() == sizeof(_value))
                std::memcpy(&_value, msg.data.data(), sizeof(_value));
        }
    }

    template<class T>
    bool Value<T>::IsEnable() const
    {
        return _enable;
    }

    template<class T>
    bool Value<T>::IsDisable() const
    {
        return not _enable;
    }

    template<class T>
    T Value<T>::Get() const
    {
        return _value;
    }

    template<class T>
    Type Value<T>::GetType() const
    {
        return _type;
    }

    template<class T>
    nox_msgs::Signal Value<T>::ToMsg() const
    {
        nox_msgs::Signal msg;
        msg.type = _type;
        msg.code = _enable ? 1 : 0;
        msg.description = ToString(_type) + (_enable ? " Enabled " : " Disabled ") + ToString(_value);
        msg.data.resize(sizeof(_value));
        std::memcpy(msg.data.data(), &_value, sizeof(_value));

        return msg;
    }

    std::string ToString(Type type)
    {
#define TypeToString(Name, Type) case Type: return #Name;
        switch (type)
        {
            SIGNAL_BOOL_TYPE_INSTANTIATION(TypeToString)
            SIGNAL_VALUE_TYPE_INSTANTIATION(TypeToString)
            SIGNAL_ENUM_TYPE_INSTANTIATION(TypeToString)
            default: return "unknown";
        }
#undef TypeToString
    }

    std::string ToString(bool value)
    {
        return value ? "true" : "false";
    }

    std::string ToString(double value)
    {
        return std::to_string(value);
    }

    std::string ToString(GearState value)
    {
        switch (value)
        {
            case P:  return "P";
            case R:  return "R";
            case N:  return "N";
            case G1: return "G1";
            case G2: return "G2";
            case G3: return "G3";
            case G4: return "G4";
            case G5: return "G5";
            case G6: return "G6";
            case D:  return "D";
            default: return "unknown";
        }
    }

    std::string ToString(BucketState value)
    {
        switch (value)
        {
            case HOLD: return "hold";
            case LIFT: return "lift";
            case DROP: return "drop";
            default:   return "unknown";
        }
    }

    std::string ToString(BucketPositionState value)
    {
        switch (value)
        {
            case BOTTOM:  return "bottom";
            case MIDDLE:  return "middle";
            case TOP:     return "top";
            case BLOCKED: return "blocked";
            default:      return "unknown";
        }
    }

    nox_msgs::SignalArray Panel::ToMsgs() const
    {
        nox_msgs::SignalArray msgs;
        msgs.header.stamp = ros::Time::now();

#define WidgetToMsg(Name, Type) msgs.signals.emplace_back(Name.ToMsg());
        SIGNAL_BOOL_TYPE_INSTANTIATION(WidgetToMsg)
        SIGNAL_VALUE_TYPE_INSTANTIATION(WidgetToMsg)
        SIGNAL_ENUM_TYPE_INSTANTIATION(WidgetToMsg)
#undef WidgetToMsg

        return msgs;
    }

    void Panel::FromMsgs(const nox_msgs::SignalArray &msgs)
    {
#define WidgetFromMsg(Name, Type) \
        case Type: Name.FromMsg(msg); break;

        for(auto & msg : msgs.signals)
        {
            switch (msg.type)
            {
                SIGNAL_BOOL_TYPE_INSTANTIATION(WidgetFromMsg)
                SIGNAL_VALUE_TYPE_INSTANTIATION(WidgetFromMsg)
                SIGNAL_ENUM_TYPE_INSTANTIATION(WidgetFromMsg)
            }
        }
#undef WidgetFromMsg
    }

    void Panel::Reset()
    {
#define WidgetReset(Name, Type) Name.Set(0);
#define EnumReset(Name, Type) Name.Set(Name##State(0));
        SIGNAL_BOOL_TYPE_INSTANTIATION(WidgetReset)
        SIGNAL_VALUE_TYPE_INSTANTIATION(WidgetReset)
        SIGNAL_ENUM_TYPE_INSTANTIATION(EnumReset)
#undef EnumReset
#undef WidgetReset
    }

    void Panel::Enable()
    {
#define WidgetEnable(Name, Type) Name.Enable();
        SIGNAL_BOOL_TYPE_INSTANTIATION(WidgetEnable)
        SIGNAL_VALUE_TYPE_INSTANTIATION(WidgetEnable)
        SIGNAL_ENUM_TYPE_INSTANTIATION(WidgetEnable)
#undef WidgetEnable
    }

    void Panel::Disable()
    {
#define WidgetDisable(Name, Type) Name.Disable();
        SIGNAL_BOOL_TYPE_INSTANTIATION(WidgetDisable)
        SIGNAL_VALUE_TYPE_INSTANTIATION(WidgetDisable)
        SIGNAL_ENUM_TYPE_INSTANTIATION(WidgetDisable)
#undef WidgetDisable
    }

}

//#undef SIGNAL_BOOL_TYPE_INSTANTIATION
//#undef SIGNAL_VALUE_TYPE_INSTANTIATION
//#undef SIGNAL_ENUM_TYPE_INSTANTIATION
