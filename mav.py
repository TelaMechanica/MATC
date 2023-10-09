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

MULTICAST_ADDRESS = "239.2.3.1"
PORT = 6969
mavlink_connection = mavutil.mavlink_connection("/dev/serial0", baud=115200)

target_callsign = None
seen_callsigns = set()
callsign_lock = threading.Lock()

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

def time_boot_ms_calculation():
    return int(((time.time() - app_start_time) * 1000) % 2147483647)
	
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
		
def send_gps_status_message(mavlink_connection):
    satellites_visible = 10
    satellite_prn = [0] * satellites_visible  # placeholder PRN values
    satellite_used = [1] * satellites_visible  # assuming all satellites are used
    satellite_elevation = [45] * satellites_visible  # placeholder elevation values
    satellite_azimuth = [255] * satellites_visible  # placeholder azimuth values
    satellite_snr = [0] * satellites_visible  # placeholder SNR values

    mavlink_connection.mav.gps_status_send(
        satellites_visible,
        satellite_prn,
        satellite_used,
        satellite_elevation,
        satellite_azimuth,
        satellite_snr
    )

def send_mavlink_messages(mavlink_connection, lat, lon, alt):
    time_boot_ms_value = time_boot_ms_calculation()
    
    # Send GLOBAL_POSITION_INT message
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

    # Send GPS_RAW_INT message
    # Constants for the GPS_RAW_INT message
    fix_type = 3  # Assume GPS_FIX_TYPE_3D_FIX for this example
    eph = 100  # Positional accuracy in cm, set to a default value
    epv = 100  # Altitude accuracy in cm, set to a default value
    vel = 0  # Ground speed in cm/s
    cog = 0  # Course over ground in centidegrees
    satellites_visible = 10  # Number of visible satellites

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


def process_cot_xml(cot_xml, mavlink_connection, target_callsign=None):
    try:
        root = ET.fromstring(cot_xml)
        callsign_element = root.find(".//contact")
        
        if callsign_element is not None and "callsign" in callsign_element.attrib:
            callsign = callsign_element.attrib["callsign"]
            with callsign_lock:
                seen_callsigns.add(callsign)
        else:
            print("Error: <contact> element or callsign attribute missing in CoT XML")
            print(f"CoT XML: {cot_xml}")  # Print the problematic XML content
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
        
        time_boot_ms_value = time_boot_ms_calculation()

        #send_heartbeatpr
        send_mavlink_messages(mavlink_connection, lat, lon, alt)


    except ET.ParseError:
        print(f"Failed to parse CoT XML: {cot_xml}")
    except KeyError as ke:
        print(str(ke))
    except ValueError as ve:
        print(str(ve))
    except Exception as e:
        print(f"Error during CoT XML conversion: {e}")
        
def process_protobuf(protobuf_data, mavlink_connection, target_callsign=None):
    try:
        decoded_message = parseProto(protobuf_data)
        #print(f"Full Protobuf Decoded Message: {decoded_message}")
        xml_detail_str = '<root>' + decoded_message.cotEvent.detail.xmlDetail + '</root>'
        root = ET.fromstring(xml_detail_str)

        # Extract callsign directly from the protobuf message
        if decoded_message.cotEvent.HasField("detail") and decoded_message.cotEvent.detail.HasField("contact"):
            callsign = decoded_message.cotEvent.detail.contact.callsign
        else:
            # Check if callsign is available inside XML detail
            contact_element = root.find('contact')
            if contact_element is not None:
                callsign = contact_element.get('callsign')
            else:
                print("Error: <contact> element missing in xmlDetail")
                #
        with callsign_lock:
            seen_callsigns.add(callsign)

        if target_callsign and callsign != target_callsign:
            return

        if not target_callsign:
            return

        lat = int(decoded_message.cotEvent.lat * 1e7)
        lon = int(decoded_message.cotEvent.lon * 1e7)
        alt = int(decoded_message.cotEvent.hae * 1e3)
        relative_alt = alt
        
        time_boot_ms_value = time_boot_ms_calculation()

        #send_heartbeat()
        send_mavlink_messages(mavlink_connection, lat, lon, alt)

    except Exception as e:
        print(f"Error during protobuf conversion: {e}")



def listen_for_messages():
    for message in listen_for_multicast_messages(MULTICAST_ADDRESS, PORT):
        #print(f"Received message: {message}")  # This line prints every message received.
        try:
            message_str = message.decode('utf-8')
            if "<event" in message_str:
                process_cot_xml(message_str, mavlink_connection, target_callsign)
            else:
                process_protobuf(message, mavlink_connection, target_callsign)
        except UnicodeDecodeError:
            try:
                process_protobuf(message, mavlink_connection, target_callsign)
            except Exception as e:
                print(f"Failed to process message as Protobuf: {e}")
        except Exception as e:
            print(f"Failed to process message: {e}")



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

    # Start the listener thread to listen for incoming messages
    listener_thread = threading.Thread(target=listen_for_messages)
    listener_thread.start()

    # Start the heartbeat thread to send heartbeats periodically
    #heartbeat_thread = threading.Thread(target=send_heartbeat_periodically)
    #heartbeat_thread.start()

    # Start the Flask app
    app.run(host='0.0.0.0', port=8080)
