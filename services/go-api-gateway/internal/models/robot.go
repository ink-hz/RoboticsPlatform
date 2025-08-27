package models

import (
	"time"
)

// Robot 机器人数据模型
type Robot struct {
	ID          uint      `json:"id" gorm:"primaryKey"`
	RobotID     string    `json:"robot_id" gorm:"uniqueIndex;not null"`
	Name        string    `json:"name" gorm:"not null"`
	Type        string    `json:"type"`
	Status      string    `json:"status" gorm:"default:offline"`
	Battery     float64   `json:"battery"`
	LastSeen    time.Time `json:"last_seen"`
	Position    Position  `json:"position" gorm:"embedded"`
	CreatedAt   time.Time `json:"created_at"`
	UpdatedAt   time.Time `json:"updated_at"`

	// 关联关系
	Telemetries []Telemetry `json:"-" gorm:"foreignKey:RobotID;references:RobotID"`
}

// Position 位置信息
type Position struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// RobotStatus 机器人状态常量
const (
	RobotStatusOnline    = "online"
	RobotStatusOffline   = "offline"
	RobotStatusError     = "error"
	RobotStatusCharging  = "charging"
	RobotStatusMaintain  = "maintenance"
)

// UpdateLastSeen 更新最后在线时间
func (r *Robot) UpdateLastSeen() {
	r.LastSeen = time.Now()
}

// SetOnline 设置机器人为在线状态
func (r *Robot) SetOnline() {
	r.Status = RobotStatusOnline
	r.UpdateLastSeen()
}

// SetOffline 设置机器人为离线状态
func (r *Robot) SetOffline() {
	r.Status = RobotStatusOffline
}

// IsOnline 检查机器人是否在线
func (r *Robot) IsOnline() bool {
	return r.Status == RobotStatusOnline
}

// GetBatteryStatus 获取电池状态描述
func (r *Robot) GetBatteryStatus() string {
	if r.Battery >= 80 {
		return "excellent"
	} else if r.Battery >= 60 {
		return "good"
	} else if r.Battery >= 40 {
		return "fair"
	} else if r.Battery >= 20 {
		return "low"
	} else {
		return "critical"
	}
}