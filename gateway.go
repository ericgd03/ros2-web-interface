package main

import (
	"context"
	"encoding/json"
	"encoding/base64"
	"log"
	"net/http"
	"time"

	"google.golang.org/grpc"
	pb "web-interface/protos" // Import your generated gRPC Go code
	"google.golang.org/protobuf/types/known/emptypb"
	// import "gateway/protos"
)

var client pb.RPCDemoClient

func main() {
	// Connect to the gRPC server running on localhost:50051
	conn, err := grpc.Dial("localhost:7042", grpc.WithInsecure())
	if err != nil {
		log.Fatalf("Could not connect to gRPC server: %v", err)
	}
	defer conn.Close()

	// Create the client stub for RobotData service
	client = pb.NewRPCDemoClient(conn)

	// Set up HTTP handlers
    http.HandleFunc("/coords", getCoordsHandler)
	http.HandleFunc("/image", imageHandler)
	http.HandleFunc("/trailer", trailerHandler)
	http.HandleFunc("/velocity", velocityHandler)

	log.Println("REST Gateway listening on :8000")
	http.ListenAndServe(":8000", nil)
}

func getCoordsHandler(w http.ResponseWriter, r *http.Request) {
    ctx, cancel := context.WithTimeout(context.Background(), time.Second)
    defer cancel()

    coords, err := client.GetMultCoords(ctx, &emptypb.Empty{})
    if err != nil {
        http.Error(w, err.Error(), http.StatusInternalServerError)
        return
    }

    json.NewEncoder(w).Encode(coords)
}

func imageHandler(w http.ResponseWriter, r *http.Request) {
    ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
    defer cancel()

    resp, err := client.GetImage(ctx, &pb.ImageRequest{})
    if err != nil {
        http.Error(w, err.Error(), http.StatusInternalServerError)
        return
    }

    // Encode image bytes to base64 string
    base64Image := base64.StdEncoding.EncodeToString(resp.ImageData)

    // Build JSON response
    json.NewEncoder(w).Encode(map[string]string{
        "image_base64": base64Image,
        "format":       resp.Format,
    })
}

func trailerHandler(w http.ResponseWriter, r *http.Request) {
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()

	resp, err := client.GetTrailer(ctx, &emptypb.Empty{})
	if err != nil {
		http.Error(w, "Failed to get trailer", http.StatusInternalServerError)
		return
	}

	json.NewEncoder(w).Encode(map[string]string{
		"trailer_number": resp.TrailerNumber,
	})
}

func velocityHandler(w http.ResponseWriter, r *http.Request) {
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()

	resp, err := client.GetVelocity(ctx, &emptypb.Empty{})
	if err != nil {
		http.Error(w, "Failed to get velocity", http.StatusInternalServerError)
		return
	}

	json.NewEncoder(w).Encode(map[string]float32{
		"linear_velocity":  resp.LinearVelocity,
		"angular_velocity": resp.AngularVelocity,
	})
}


