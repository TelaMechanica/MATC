MATC: Modular Antenna Tracker Caddy

Overview
The Modular Antenna Tracker Caddy (MATC) project emphasizes the principles of Open Source and Open Architecture. These principles culminate in a highly flexible system capable of tracking any position event inside the Team Awareness Kit (TAK) environment, encompassing UAS, MANET radios, and Android devices.

Hardware Features

Flexibility: MATC can be tailored to track any position event within the TAK environment.

Single Board Computer Utilization: MATC uses a Raspberry Pi powered by a Power Distribution Board (PDB). Upon startup, the system checks for an Ethernet connection and ensures it can communicate with the MATC service. The MATC server will listen on a specified Multicast Address and track devices.

Data Conversion: The software can convert between the TAK XML and Protobuf format into the MAVLink XML Schema, facilitating integration with various systems. This includes translating a username's location from TAK syntax into a MAVLink message and supporting devices like the MFD Mini Crossbow AAT.

MAVLink Connection: Connects and interacts with MAVLink compatible devices using pymavlink.  

  Dual Multicast Listeners: The program listens to multicast messages on two different addresses and ports:

 ** Default TAK**: 239.2.3.1:6969
 ** Persistent Systems**: 239.23.212.230:18999
 
  CoT XML and Protocol Buffers Message Parsing: The application can process messages in both CoT XML format and Protocol Buffers format, determining the correct parsing method based on   the message contents.

  Flask Web Interface: Provides a web-based interface for users to interact with the system, allowing for the selection of a target callsign.

Expandable: Comments within the server script and frontend .html provide insights for developers. The system allows for easy expansion and integration with tools like Chat GPT for added functionalities.

The Case for Modularity

MATC emphasizes modularity, providing a robust and flexible foundation for developers and users to tailor the system to their needs. The system utilizes a Raspberry Pi, ensuring easy scalability and modifications. Tracking isn't limited to just UAS via the UAS Tool but extends to other devices in the TAK network, making it a versatile tool for diverse applications.

Technical Details
MATC listens for UDP multicast messages at the specified address and port, processing and optionally reformatting them based on user input in the HTML frontend.

Conversion operations utilize libraries like pymavlink, mavutil, takprotobuf, and parseProto. The AAT uses local serial GPS data, and if it's actively receiving messages from the MATC server, tracking will commence. Moreover, tracking capabilities span across various devices and platforms.

Hardware Specifications
The MFD Mini Crossbow AAT is a notable hardware utilized with the MATC system. It supports MAVLink versions V1.0 and V2.0, as well as VBI Downlink. With a weight of 470g and torque specifications provided, it is available at an MSRP of $150.

**Running the Program**
To run the application, execute the main script. The application will start, initializing the MAVLink connection and creating listener threads for each multicast address. The Flask web server will then start, and the application will be accessible via a web browser at http://0.0.0.0:8080.

Collaboration
For collaboration, queries, or further information, please contact: antracker@proton.me
