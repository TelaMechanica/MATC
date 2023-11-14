import socket
import struct
import threading
import time
import subprocess
import xml.etree.ElementTree as ET
from flask import Flask, render_template, request, jsonify
from pymavlink import mavutil
from takprotobuf import parseProto

app = Flask(__name__)

# Define your multicast addresses and ports
MULTICAST_ADDRESS1 = "239.23.212.230"
PORT1 = 18999
MULTICAST_ADDRESS2 = "239.2.3.1"
PORT2 = 6969

# Create MAVLink connection
mavlink_connection = mavutil.mavlink_connection("/dev/serial0", baud=115200)

# Global variables to store the last MAVLink message and the time it was sent
last_mavlink_message = None
last_message_time = 0

# Define global variables for callsign and seen_callsigns
target_callsign = None
seen_callsigns = set()
callsign_lock = threading.Lock()



# Create a dictionary to associate callsigns with their sources (sockets)
callsign_sources = {}

# Define a function to listen for multicast messages on a given address and port
def listen_for_multicast_messages(address, port):
    multicast_group = (address, port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(multicast_group)

    group = socket.inet_aton(address)
    mreq = struct.pack('4sL', group, socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    while True:
        message, _ = sock.recvfrom(4096)
        yield message

# Define a function to calculate time_boot_ms
def time_boot_ms_calculation():
    return int(((time.time() - app_start_time) * 1000) % 2147483647)

# Define a function to send heartbeat
def send_heartbeat():
    base_mode = mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
    custom_mode = 0
    system_status = mavutil.mavlink.MAV_STATE_ACTIVE
    mavlink_version = 3

    mavlink_connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GENERIC,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        base_mode,
        custom_mode,
        system_status,
        mavutil.mavlink.MAV_MODE_STABILIZE_ARMED,
        mavlink_version
    )

# Define a function to send heartbeat periodically
def send_heartbeat_periodically():
    while True:
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
        custom_mode = 0
        system_status = mavutil.mavlink.MAV_STATE_ACTIVE
        mavlink_version = 3

        mavlink_connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            base_mode,
            custom_mode,
            system_status,
            mavutil.mavlink.MAV_MODE_STABILIZE_ARMED,
            mavlink_version
        )
        time.sleep(1)

# Define a function to send GPS status message
def send_gps_status_message(mavlink_connection):
    satellites_visible = 10
    satellite_prn = [0] * satellites_visible
    satellite_used = [1] * satellites_visible
    satellite_elevation = [45] * satellites_visible
    satellite_azimuth = [255] * satellites_visible
    satellite_snr = [0] * satellites_visible

    mavlink_connection.mav.gps_status_send(
        satellites_visible,
        satellite_prn,
        satellite_used,
        satellite_elevation,
        satellite_azimuth,
        satellite_snr
    )

# Define a function to send MAVLink messages
def send_mavlink_messages(mavlink_connection, lat, lon, alt): 
    global last_mavlink_message, last_message_time
    time_boot_ms_value = time_boot_ms_calculation()

    mavlink_connection.mav.global_position_int_send(
        time_boot_ms=time_boot_ms_value,
        lat=lat,
        lon=lon,
        alt=alt,
        relative_alt=alt,
        vx=0,
        vy=0,
        vz=0,
        hdg=0
    )

    fix_type = 3
    eph = 100
    epv = 100
    vel = 0
    cog = 0
    satellites_visible = 10

    mavlink_connection.mav.gps_raw_int_send(
        time_usec=int(time.time() * 1e6),
        fix_type=fix_type,
        lat=lat,
        lon=lon,
        alt=alt,
        eph=eph,
        epv=epv,
        vel=vel,
        cog=cog,
        satellites_visible=satellites_visible
    )
      
# After sending the message, update the last message and time
    last_mavlink_message = (lat, lon, alt)
    last_message_time = time.time()  


def resend_last_mavlink_message():
    global last_mavlink_message, last_message_time

    while True:
        current_time = time.time()
        if last_mavlink_message and (current_time - last_message_time) > 1:
            lat, lon, alt = last_mavlink_message
            send_mavlink_messages(mavlink_connection, lat, lon, alt)
        time.sleep(1)  # Check every second


    
# Define a function to process CoT XML messages
def process_cot_xml(cot_xml, mavlink_connection, target_callsign=None, source_socket=None):
    try:
        # Print the original XML message
        print(f"Received CoT XML: {cot_xml}")
        
        root = ET.fromstring(cot_xml)
        callsign_element = root.find(".//contact")

        if callsign_element is not None and "callsign" in callsign_element.attrib:
            callsign = callsign_element.attrib["callsign"]
            with callsign_lock:
                seen_callsigns.add(callsign)
        else:
            print("Error: <contact> element or callsign attribute missing in CoT XML")
            print(f"CoT XML: {cot_xml}")
            return

        if target_callsign and callsign != target_callsign:
            return

        if not target_callsign:
            return

        point = root.find(".//point")
        if point is None:
            raise ValueError("Missing <point> element in XML.")

        if 'lat' in point.attrib and 'lon' in point.attrib:
            lat = int(float(point.attrib['lat']) * 1e7)
            lon = int(float(point.attrib['lon']) * 1e7)
        else:
            raise KeyError("Missing expected attribute in <point> element.")

        alt = int(float(point.attrib.get('hae', 0)) * 1e3)
        relative_alt = alt

        # Print the converted MAVLink message details
        print(f"Sending MAVLink message: Lat: {lat}, Lon: {lon}, Alt: {alt}")

        send_mavlink_messages(mavlink_connection, lat, lon, alt)

    except ET.ParseError:
        print(f"Failed to parse CoT XML: {cot_xml}")
    except KeyError as ke:
        print(str(ke))
    except ValueError as ve:
        print(str(ve))
    except Exception as e:
        print(f"Error during CoT XML conversion: {e}")

# Define a function to process Protocol Buffers
def process_protobuf(protobuf_data, mavlink_connection, target_callsign=None, source_socket=None):
    try:
        decoded_message = parseProto(protobuf_data)
        xml_detail_str = '<root>' + decoded_message.cotEvent.detail.xmlDetail + '</root>'
        root = ET.fromstring(xml_detail_str)

        if decoded_message.cotEvent.HasField("detail") and decoded_message.cotEvent.detail.HasField("contact"):
            callsign = decoded_message.cotEvent.detail.contact.callsign
        else:
            contact_element = root.find('contact')
            if contact_element is not None:
                callsign = contact_element.get('callsign')
            else:
                print("Error: <contact> element missing in xmlDetail")

        with callsign_lock:
            seen_callsigns.add(callsign)

        if target_callsign and callsign != target_callsign:
            return

        lat = int(decoded_message.cotEvent.lat * 1e7)
        lon = int(decoded_message.cotEvent.lon * 1e7)
        alt = int(decoded_message.cotEvent.hae * 1e3)
        relative_alt = alt

        time_boot_ms_value = time_boot_ms_calculation()

        send_mavlink_messages(mavlink_connection, lat, lon, alt)

    except Exception as e:
        print(f"Error during protobuf conversion: {e}")

# Define a function to listen for messages on a given socket
def listen_for_messages(address, port):
    for message in listen_for_multicast_messages(address, port):
        try:
            message_str = message.decode('utf-8')
            if "<event" in message_str:
                #print(f"Received XML message: {message_str}")  # Print the XML message
                process_cot_xml(message_str, mavlink_connection, target_callsign, (address, port))
            else:
                process_protobuf(message, mavlink_connection, target_callsign, (address, port))
        except UnicodeDecodeError:
            try:
                process_protobuf(message, mavlink_connection, target_callsign, (address, port))
            except Exception as e:
                print(f"Failed to process message as Protobuf: {e}")
        except Exception as e:
            print(f"Failed to process message: {e}")

# Define the Flask routes and front-end code (unchanged)

@app.route('/', methods=['GET', 'POST'])
def index():
    global target_callsign
    if request.method == 'POST':
        target_callsign = request.form.get('callsign')
        
        # Placeholder to indicate success/failure in callsign selection
        if target_callsign:
            print(f"Selected callsign: {target_callsign}")
        else:
            print("Callsign was not selected.")

    with callsign_lock:
        callsign_list = list(seen_callsigns)

    return render_template('index.html', callsigns=callsign_list, selected_callsign=target_callsign)

@app.route('/callsigns', methods=['GET'])
def get_callsigns():
    with callsign_lock:
        callsign_list = list(seen_callsigns)
    return jsonify(callsigns=callsign_list)

if __name__ == '__main__':
    # Set reference time for the application
    app_start_time = time.time()
    print(f"Application start time (in seconds since the epoch): {app_start_time}")

    # Start the first listener thread for the first multicast socket
    listener_thread1 = threading.Thread(target=listen_for_messages, args=(MULTICAST_ADDRESS1, PORT1))
    listener_thread1.start()

    # Start the second listener thread for the second multicast socket
    listener_thread2 = threading.Thread(target=listen_for_messages, args=(MULTICAST_ADDRESS2, PORT2))
    listener_thread2.start()

    # Start a thread to resend the last MAVLink message if needed
    resend_thread = threading.Thread(target=resend_last_mavlink_message)
    resend_thread.start()

    # Start the Flask app
    app.run(host='0.0.0.0', port=8080, threaded=True)

