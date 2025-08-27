package models

import (
	"database/sql/driver"
	"encoding/json"
	"errors"
	"time"
)

// Telemetry 遥测数据模型
type Telemetry struct {
	ID        uint      `json:"id" gorm:"primaryKey"`
	RobotID   string    `json:"robot_id" gorm:"not null;index"`
	Data      JSON      `json:"data" gorm:"type:jsonb"`
	Timestamp time.Time `json:"timestamp" gorm:"not null;index"`
	CreatedAt time.Time `json:"created_at"`

	// 关联
	Robot Robot `json:"-" gorm:"foreignKey:RobotID;references:RobotID"`
}

// JSON 自定义JSON类型，支持PostgreSQL的jsonb
type JSON map[string]interface{}

// Value 实现driver.Valuer接口，用于数据库存储
func (j JSON) Value() (driver.Value, error) {
	if j == nil {
		return nil, nil
	}
	return json.Marshal(j)
}

// Scan 实现sql.Scanner接口，用于数据库读取
func (j *JSON) Scan(value interface{}) error {
	if value == nil {
		*j = nil
		return nil
	}

	var bytes []byte
	switch v := value.(type) {
	case []byte:
		bytes = v
	case string:
		bytes = []byte(v)
	default:
		return errors.New("type assertion to []byte failed")
	}

	return json.Unmarshal(bytes, j)
}

// TelemetryData 遥测数据结构（用于API）
type TelemetryData struct {
	Battery    float64                `json:"battery,omitempty"`
	Position   *Position              `json:"position,omitempty"`
	Velocity   *Velocity              `json:"velocity,omitempty"`
	Sensors    map[string]interface{} `json:"sensors,omitempty"`
	Status     string                 `json:"status,omitempty"`
	Timestamp  int64                  `json:"timestamp,omitempty"`
	Extra      map[string]interface{} `json:"extra,omitempty"`
}

// Velocity 速度信息
type Velocity struct {
	Linear  float64 `json:"linear"`
	Angular float64 `json:"angular"`
}

// NewTelemetry 创建新的遥测数据记录
func NewTelemetry(robotID string, data map[string]interface{}) *Telemetry {
	return &Telemetry{
		RobotID:   robotID,
		Data:      JSON(data),
		Timestamp: time.Now(),
		CreatedAt: time.Now(),
	}
}

// GetBattery 获取电池数据
func (t *Telemetry) GetBattery() float64 {
	if battery, ok := t.Data["battery"].(float64); ok {
		return battery
	}
	return 0
}

// GetPosition 获取位置数据
func (t *Telemetry) GetPosition() *Position {
	if posData, ok := t.Data["position"].(map[string]interface{}); ok {
		pos := &Position{}
		if x, ok := posData["x"].(float64); ok {
			pos.X = x
		}
		if y, ok := posData["y"].(float64); ok {
			pos.Y = y
		}
		if z, ok := posData["z"].(float64); ok {
			pos.Z = z
		}
		return pos
	}
	return nil
}

// GetStatus 获取状态数据
func (t *Telemetry) GetStatus() string {
	if status, ok := t.Data["status"].(string); ok {
		return status
	}
	return ""
}

// TableName 指定表名
func (Telemetry) TableName() string {
	return "telemetries"
}