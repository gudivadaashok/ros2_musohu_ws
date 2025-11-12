#!/bin/bash
#===============================================================================
# Musohu Helmet Data Recording System
#===============================================================================
# Description: ROS2 bag recording script for helmet sensor suite
# Author: Musohu RobotiXX Team
# Version: 2.0
# Date: November 2025
#
# Hardware: NVIDIA Jetson with ZED 2i, ReSpeaker, RS LiDAR, Witmotion IMU
# 
# Usage: 
#   ./record_helmet_data.sh [OPTIONS]
#   
# Options:
#   -d, --duration SECONDS    Recording duration (default: 30)
#   -o, --output PATH         Output directory (default: ~/helmet_bags)
#   -n, --name PREFIX         Bag name prefix (default: helmet_complete)
#   -c, --compress            Enable bag compression
#   -v, --verbose             Verbose output
#   -h, --help                Show this help
#
# Examples:
#   ./record_helmet_data.sh -d 60 -c              # 60 seconds, compressed
#   ./record_helmet_data.sh --name test_run -v    # Custom name, verbose
#===============================================================================

set -euo pipefail  # Exit on error, undefined vars, pipe failures

# Script configuration
readonly SCRIPT_NAME="$(basename "$0")"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_DIR="$SCRIPT_DIR"
readonly VERSION="2.0"

# Default parameters
DEFAULT_DURATION=30
DEFAULT_OUTPUT_DIR="$HOME/helmet_bags"
DEFAULT_NAME_PREFIX="helmet_complete"
DEFAULT_COMPRESS=false
DEFAULT_VERBOSE=false

# Global variables
DURATION="$DEFAULT_DURATION"
OUTPUT_DIR="$DEFAULT_OUTPUT_DIR"
NAME_PREFIX="$DEFAULT_NAME_PREFIX"
COMPRESS="$DEFAULT_COMPRESS"
VERBOSE="$DEFAULT_VERBOSE"

# Color codes for output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly PURPLE='\033[0;35m'
readonly CYAN='\033[0;36m'
readonly NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $*"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $*"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2
}

log_debug() {
    if [[ "$VERBOSE" == "true" ]]; then
        echo -e "${PURPLE}[DEBUG]${NC} $*"
    fi
}

# Help function
show_help() {
    cat << EOF
${SCRIPT_NAME} v${VERSION} - Musohu Helmet Data Recording System

USAGE:
    ${SCRIPT_NAME} [OPTIONS]

OPTIONS:
    -d, --duration SECONDS     Recording duration in seconds (default: ${DEFAULT_DURATION})
    -o, --output PATH          Output directory (default: ${DEFAULT_OUTPUT_DIR})
    -n, --name PREFIX          Bag filename prefix (default: ${DEFAULT_NAME_PREFIX})
    -c, --compress             Enable bag compression (reduces file size)
    -v, --verbose              Enable verbose output
    -h, --help                 Show this help message

EXAMPLES:
    ${SCRIPT_NAME}                                    # Quick 30-second recording
    ${SCRIPT_NAME} -d 120 -c                         # 2-minute compressed recording
    ${SCRIPT_NAME} --duration 60 --name experiment1  # Custom name and duration
    ${SCRIPT_NAME} -d 300 -o /data/bags -v           # Long recording to custom path

SENSOR TOPICS RECORDED:
    • ZED Camera: RGB, Depth, IMU, Odometry, Pose
    • Audio: 6-channel microphone array + DOA + Speech detection  
    • LiDAR: 3D point cloud data
    • IMU: Calibration and orientation data
    • System: Transforms, diagnostics, logging

For more information, visit: https://github.com/gudivadaashok/ros2_musohu_ws
EOF
}

# Parse command line arguments
parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -d|--duration)
                if [[ -n "${2:-}" && "$2" =~ ^[0-9]+$ ]]; then
                    DURATION="$2"
                    shift 2
                else
                    log_error "Duration must be a positive integer"
                    exit 1
                fi
                ;;
            -o|--output)
                if [[ -n "${2:-}" ]]; then
                    OUTPUT_DIR="$2"
                    shift 2
                else
                    log_error "Output directory cannot be empty"
                    exit 1
                fi
                ;;
            -n|--name)
                if [[ -n "${2:-}" ]]; then
                    NAME_PREFIX="$2"
                    shift 2
                else
                    log_error "Name prefix cannot be empty"
                    exit 1
                fi
                ;;
            -c|--compress)
                COMPRESS=true
                shift
                ;;
            -v|--verbose)
                VERBOSE=true
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                # Legacy support: first argument as duration
                if [[ "$1" =~ ^[0-9]+$ ]]; then
                    DURATION="$1"
                    shift
                else
                    log_error "Unknown option: $1"
                    echo "Use --help for usage information"
                    exit 1
                fi
                ;;
        esac
    done
}

# Validate environment
validate_environment() {
    log_info "Validating environment..."
    
    # Check if we're in a ROS2 workspace
    if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
        log_error "ROS2 workspace not found at ${WORKSPACE_DIR}"
        log_error "Please run this script from the workspace root directory"
        exit 1
    fi
    
    # Check available disk space (warn if less than 1GB)
    local available_space
    available_space=$(df "${OUTPUT_DIR%/*}" 2>/dev/null | awk 'NR==2 {print $4}' || echo "0")
    if [[ "$available_space" -lt 1048576 ]]; then  # 1GB in KB
        log_warning "Low disk space: $(( available_space / 1024 ))MB available"
        log_warning "Recording large sensor data may fill the disk"
    fi
    
    log_debug "Environment validation complete"
}

# Source ROS2 workspace
source_workspace() {
    log_info "Sourcing ROS2 workspace..."
    
    # Temporarily disable strict mode for ROS2 setup scripts (they have unbound variables)
    set +u
    
    # Source ROS2 installation
    if [[ -f "/opt/ros/humble/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "/opt/ros/humble/setup.bash"
        log_debug "Sourced ROS2 Humble"
    else
        log_error "ROS2 installation not found"
        set -u  # Re-enable strict mode
        exit 1
    fi
    
    # Source workspace
    # shellcheck disable=SC1091
    source "${WORKSPACE_DIR}/install/setup.bash"
    log_debug "Sourced workspace: ${WORKSPACE_DIR}"
    
    # Re-enable strict mode
    set -u
    
    # Verify ROS2 is working
    if ! command -v ros2 >/dev/null 2>&1; then
        log_error "ROS2 command not found after sourcing"
        exit 1
    fi
    
    log_success "ROS2 workspace sourced successfully"
}

# Check ROS2 daemon and topics
validate_ros_environment() {
    log_info "Validating ROS2 environment..."
    
    # Check if ROS2 daemon is running
    if ! ros2 daemon status >/dev/null 2>&1; then
        log_info "Starting ROS2 daemon..."
        ros2 daemon start
        sleep 2
    fi
    
    # Get available topics
    local available_topics
    if ! available_topics=$(ros2 topic list 2>/dev/null); then
        log_error "Failed to get topic list. Is ROS2 running?"
        exit 1
    fi
    
    # Define critical topics (must exist)
    local critical_topics=(
        "/tf"
        "/tf_static"
    )
    
    # Define sensor topics (warn if missing)
    local sensor_topics=(
        "/zed2i/zed_node/rgb/color/rect/image/compressed"
        "/zed2i/zed_node/depth/depth_registered/compressed"
        "/audio"
        "/rslidar_points"
        "/imu_node/cali"
    )
    
    # Check critical topicsros2 bag info ~/helmet_bags/helmet_complete_YYYYMMDD_HHMMSS


    local missing_critical=()
    for topic in "${critical_topics[@]}"; do
        if ! echo "$available_topics" | grep -q "^$topic$"; then
            missing_critical+=("$topic")
        fi
    done
    
    if [[ ${#missing_critical[@]} -gt 0 ]]; then
        log_error "Critical topics missing:"
        printf '  - %s\n' "${missing_critical[@]}"
        log_error "Cannot proceed without critical topics"
        exit 1
    fi
    
    # Check sensor topics
    local missing_sensors=()
    for topic in "${sensor_topics[@]}"; do
        if ! echo "$available_topics" | grep -q "^$topic$"; then
            missing_sensors+=("$topic")
        fi
    done
    
    if [[ ${#missing_sensors[@]} -gt 0 ]]; then
        log_warning "Some sensor topics are missing:"
        printf '  - %s\n' "${missing_sensors[@]}"
        log_warning "Make sure all sensors are running for complete data"
        
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            log_info "Recording cancelled by user"
            exit 0
        fi
    fi
    
    local topic_count
    topic_count=$(echo "$available_topics" | wc -l)
    log_success "ROS2 environment validated (${topic_count} topics available)"
}

# Prepare output directory
prepare_output() {
    log_info "Preparing output directory..."
    
    # Create output directory if it doesn't exist
    if ! mkdir -p "$OUTPUT_DIR"; then
        log_error "Failed to create output directory: $OUTPUT_DIR"
        exit 1
    fi
    
    # Check write permissions
    if [[ ! -w "$OUTPUT_DIR" ]]; then
        log_error "No write permission for output directory: $OUTPUT_DIR"
        exit 1
    fi
    
    log_debug "Output directory ready: $OUTPUT_DIR"
}

# Generate bag filename
generate_filename() {
    local timestamp
    timestamp=$(date +%Y%m%d_%H%M%S)
    echo "${OUTPUT_DIR}/${NAME_PREFIX}_${timestamp}"
}

# Build recording command
build_record_command() {
    local bag_path="$1"
    local cmd=(
        "ros2" "bag" "record"
        "-o" "$bag_path"
        "--max-bag-duration" "$DURATION"
    )
    
    if [[ "$COMPRESS" == "true" ]]; then
        cmd+=("--compression-mode" "file")
        cmd+=("--compression-format" "zstd")
    fi
    
    # Add all topics
    local topics=(
        # ZED Camera topics
        "/zed2i/zed_node/rgb/color/rect/image/compressed"
        "/zed2i/zed_node/depth/depth_registered/compressed"
        "/zed2i/zed_node/imu/data"
        "/zed2i/zed_node/odom"
        "/zed2i/zed_node/pose"
        
        # Audio topics
        "/audio"
        "/audio/channel0" "/audio/channel1" "/audio/channel2"
        "/audio/channel3" "/audio/channel4" "/audio/channel5"
        "/doa" "/speech_audio" "/vad"
        
        # LiDAR topics
        "/rslidar_points"
        
        # IMU topics
        "/imu_node/cali"
        
        # System topics
        "/tf" "/tf_static"
        "/diagnostics"
        "/rosout"
    )
    
    cmd+=("${topics[@]}")
    echo "${cmd[@]}"
}

# Main recording function
record_data() {
    local bag_path
    bag_path=$(generate_filename)
    
    log_info "Starting data recording..."
    log_info "Duration: ${DURATION} seconds"
    log_info "Output: ${bag_path}"
    log_info "Compression: $([ "$COMPRESS" == "true" ] && echo "enabled" || echo "disabled")"
    
    # Build and execute recording command
    local record_cmd
    record_cmd=$(build_record_command "$bag_path")
    
    log_debug "Recording command: $record_cmd"
    
    # Start recording with progress indication
    log_info "Recording in progress..."
    if [[ "$VERBOSE" == "true" ]]; then
        eval "$record_cmd"
    else
        eval "$record_cmd" 2>/dev/null
    fi
    
    # Verify bag was created
    if [[ ! -d "$bag_path" ]]; then
        log_error "Recording failed - bag directory not created"
        exit 1
    fi
    
    # Get bag info
    local bag_size
    bag_size=$(du -sh "$bag_path" 2>/dev/null | cut -f1 || echo "unknown")
    
    log_success "Recording completed successfully!"
    log_success "Bag size: $bag_size"
    log_success "Location: $bag_path"
    
    # Display post-recording information
    echo
    log_info "Post-recording commands:"
    echo "  Inspect bag:  ros2 bag info '$bag_path'"
    echo "  Play bag:     ros2 bag play '$bag_path'"
    echo "  Bag contents: ros2 bag info '$bag_path' --verbose"
    
    if [[ "$COMPRESS" == "false" ]]; then
        echo "  Compress bag: ros2 bag reindex '$bag_path' --compression-format zstd"
    fi
    
    return 0
}

# Cleanup function
cleanup() {
    local exit_code=$?
    if [[ $exit_code -ne 0 ]]; then
        log_error "Recording failed with exit code $exit_code"
    fi
    exit $exit_code
}

# Main function
main() {
    # Set up signal handling
    trap cleanup EXIT INT TERM
    
    # Parse arguments
    parse_arguments "$@"
    
    # Show banner
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  Musohu Helmet Data Recording v${VERSION}${NC}"
    echo -e "${CYAN}========================================${NC}"
    
    # Validation pipeline
    validate_environment
    source_workspace
    validate_ros_environment
    prepare_output
    
    # Start recording
    record_data
    
    log_success "All operations completed successfully!"
}

# Execute main function with all arguments
main "$@"