package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"log"
	"math/rand"
	"net/http"
	"time"
)

const apiURL = "http://127.0.0.1:8000/api/v1/robots"

type Position struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type Velocity struct {
	Linear  float64 `json:"linear"`
	Angular float64 `json:"angular"`
}

type TelemetryData struct {
	Battery  float64  `json:"battery"`
	Position Position `json:"position"`
	Velocity Velocity `json:"velocity"`
	Status   string   `json:"status"`
}

type TelemetryPayload struct {
	Data TelemetryData `json:"data"`
}

func main() {
	fmt.Println("🚀 Sending demo telemetry data to Robot Cloud Platform...")

	robotIDs := []string{"demo-robot-001", "demo-robot-002", "demo-robot-003"}
	statuses := []string{"active", "idle", "charging"}

	rand.Seed(time.Now().UnixNano())

	for i := 0; i < 10; i++ {
		for _, robotID := range robotIDs {
			// Generate random telemetry data
			telemetry := TelemetryPayload{
				Data: TelemetryData{
					Battery: 50 + rand.Float64()*50, // 50-100%
					Position: Position{
						X: rand.Float64() * 100,
						Y: rand.Float64() * 100,
						Z: 0,
					},
					Velocity: Velocity{
						Linear:  rand.Float64() * 2,
						Angular: rand.Float64()*2 - 1, // -1 to 1
					},
					Status: statuses[rand.Intn(len(statuses))],
				},
			}

			// Send telemetry data
			url := fmt.Sprintf("%s/%s/telemetry", apiURL, robotID)
			jsonData, err := json.Marshal(telemetry)
			if err != nil {
				log.Printf("Error marshaling data: %v", err)
				continue
			}

			resp, err := http.Post(url, "application/json", bytes.NewBuffer(jsonData))
			if err != nil {
				log.Printf("Error sending data for %s: %v", robotID, err)
				continue
			}
			defer resp.Body.Close()

			if resp.StatusCode == http.StatusOK {
				fmt.Printf("✅ Sent telemetry for %s - Battery: %.1f%%, Position: (%.1f, %.1f)\n",
					robotID, telemetry.Data.Battery, telemetry.Data.Position.X, telemetry.Data.Position.Y)
			} else {
				fmt.Printf("❌ Failed to send data for %s: Status %d\n", robotID, resp.StatusCode)
			}
		}

		time.Sleep(2 * time.Second)
	}

	fmt.Println("\n✨ Demo data sending complete!")
}