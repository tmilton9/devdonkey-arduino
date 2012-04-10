
#include "WProgram.h"
#include "WiFly.h"
#include "QueueList.h"

#define NONE 0
#define ERROR 1
#define WARN 2
#define INFO 3
#define DEBUG 4

void log(int, const char[]);
void log(int, int);
void log(int, String);
bool isTimeout();
bool readRequests(Client);
void sendResponse(Client);


enum PHASE {
	IDLE,
	READING,
	WRITING
};

/**
 * Timeout, in miliseconds, in which all requests must
 * complete. If they fail to do so, we force close.
 */
const long REQUEST_TIMEOUT_MS = 2000;

/**
 * Maximum length, in characters of a byte,
 * of HTTP requests, including headers and everything.
 */
const int REQUEST_MAX_LENGTH = 200;

/**
 * Log level.
 */
const int LOG_LEVEL = DEBUG;


/*
 * **************************************************************
 * *** You probably don't need to touch anything below this. ****
 * **************************************************************
 */

Server server(80);


enum PHASE currentStatus = IDLE;

long currentTime = millis();
long lastIter = 0;

//QueueList <char[]> requests;


void setup() {
	Serial.begin(9600);
	log(INFO, "Initializing WiFly module.");
	SpiSerial.begin();
	delay(5000);
	server.begin();
	log(INFO, "Server Ready.");
}


void loop() {

	/*
	 * Make sure that the timer hasn't overflown.
	 */
	currentTime = millis();
	if (currentTime < lastIter) {
		// There was an overflow.
		lastIter = currentTime;
		return;
	}

	// listen for incoming clients
	Client client = server.available();

	if (client) {

		log(INFO, "Server client available.");

		if (client.connected()) {

			if (readRequests(client)) {

				log(DEBUG, "Request Read was successful.");
				sendResponse(client);
				// close the connection:
				log(INFO, "Closing connection");
				client.flush();
				client.stop();

			} else {
				log(DEBUG, "Request Read failed.");
			}
		} else {
			log(DEBUG, "Client is available but not connected.");
		}
	} else {
		if (currentTime % 1000 == 0) {
			log(DEBUG, "No clients available.");
		}
		//delay(1);
		/*if (SpiSerial.available()) {
			log(DEBUG, "Force close the connection.");
			SpiSerial.print("close");
			delay(250);
			while (!(SpiSerial.available() && (SpiSerial.read() == '\n'))) {
			  // Skip remainder of response
			}
		}*/
	}

	// Update the timestamp of this pass.
	lastIter = currentTime;
}



bool readRequests(Client client) {

	// an http request ends with a blank line
	boolean currentLineIsBlank = false;

	// This is the request read buffer.
	String buffer = "";
	int buffer_position = 0;

	while (client.connected()) {

		if (client.available()) {

			unsigned char c = client.read();

			if (c == '\n' && currentLineIsBlank) {
				// if you've gotten to the end of the line (received a newline
				// character) and the line is blank, the http request has ended,
				// so you can send a reply
				log(INFO, "Adding request to queue:");
				log(INFO, buffer);
				//requests.push(buffer);
				log(DEBUG, "Flushing client stream.");
				client.flush();
				return true;
			} else {

				if (buffer_position == REQUEST_MAX_LENGTH) {
					// Buffer full. Fail.
					log(ERROR, "Request buffer full. Aborting.");
					return false;
				}

				// the request isn't finished. add to buffer.
				buffer += c;
				buffer_position++;
			}

			if (c == '\n') {
				// you're starting a new line
				currentLineIsBlank = true;
			} else if (c != '\r') {
				// you've gotten a character on the current line
				currentLineIsBlank = false;
			}

		}

		// Check request timeout.
		if (isTimeout()) {
			return false;
		}
	}
	return false;
}

void sendResponse(Client client) {

	//while (requests.count() > 0) {

		//char request[REQUEST_MAX_LENGTH];
		//request = requests.pop();

		//log(DEBUG, "Processing request:");
		//log(DEBUG, request);

		log(INFO, "Returning HTTP Headers.");
		// send a standard http response header
		client.println("HTTP/1.1 200 OK");
		client.println("Content-Type: text/html");
		client.println();

		// output the value of each analog input pin
		log(INFO, "Returning HTTP Payload.");
		client.print("<html><head><title>Robot HTTPD</title>");
		client.print(
				"<META HTTP-EQUIV=\"REFRESH\" CONTENT=\"5\"></head>");
		client.print("<body><h1>Robot HTTPD</h1>");
		//client.print("<h2>The time is ");
		//char c[10];   // simply large enough - don't forget the
		// extra byte needed for the trailing '\0'
		//sprintf(c, "%d", millis());
		//client.print(c);
		//client.print("</h2>");

		for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
			client.print("<p>");
			client.print("analog input ");
			client.print(analogChannel);
			client.print(" is ");
			client.print(analogRead(analogChannel));
			client.println("<br/>");
		}
		client.println("</p>");
		client.print("</body></html>");
		// give the web browser time to receive the data
		delay(1);
	//}
}

/**
 * Checks the REQUEST_TIMEOUT property.
 */
bool isTimeout() {

	// Check for timeouts.
	if ((currentTime - lastIter) > REQUEST_TIMEOUT_MS) {
		// close the connection:
		log(INFO, "Closing connection. Request Timeout reached.");
		log(DEBUG, "Current Time: ");
		log(DEBUG, currentTime);
		log(DEBUG, "Last Iteration: ");
		log(DEBUG, lastIter);
		log(DEBUG, "Elapsed Time: ");
		log(DEBUG, currentTime - lastIter);
		log(DEBUG, "Timeout: ");
		log(DEBUG, REQUEST_TIMEOUT_MS);

		return true;
	}
	return false;
}

void log(int level, const char message[]) {
	if (level <= LOG_LEVEL) {
		Serial.println(message);
	}
}

void log(int level, int message) {
	if (level <= LOG_LEVEL) {
		Serial.println(message);
	}
}

void log(int level, String message) {
	if (level <= LOG_LEVEL) {
		Serial.println(message);
	}
}

int main(void) {
	init();
	setup();
	for (;;)
		loop();
	return 0;
}

extern "C" void __cxa_pure_virtual() {
	cli();
	for (;;)
		;
}
