#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <time.h>

// =========================================================
// KONSTANTEN UND TYPEN (Müssen am Anfang stehen)
// =========================================================

// Der POCSAG-Standard
#define SYNC 0x7CD215D8
#define IDLE 0x7A89C197
#define FRAME_SIZE 2
#define BATCH_SIZE 16
#define PREAMBLE_LENGTH 576
#define FLAG_ADDRESS 0x000000
#define FLAG_MESSAGE 0x100000
#define TEXT_BITS_PER_WORD 20
#define TEXT_BITS_PER_CHAR 7
#define CRC_BITS 10
#define CRC_GENERATOR 0b11101101001

// Function Codes (Die letzten 2 Bits des Adressworts)
#define FLAG_FUNC_0 0x0 // Alert/Numeric
#define FLAG_FUNC_1 0x1 // Numeric
#define FLAG_FUNC_2 0x2 // Numeric
#define FLAG_FUNC_3 0x3 // Alpha (Text)
typedef uint32_t FunctionCode;

// PCM/Audio Konstanten
#define SYMRATE 38400
#define SAMPLE_RATE 22050
#define BAUD_RATE 512
#define MIN_DELAY 1
#define MAX_DELAY 10

// =========================================================
// FUNKTIONSPROTOTYPEN (Müssen vor main() stehen)
// =========================================================

uint32_t crc(uint32_t inputMsg);
uint32_t parity(uint32_t x);
uint32_t encodeCodeword(uint32_t msg);
uint32_t encodeASCII(uint32_t initial_offset, char* str, uint32_t* out);
uint32_t addressOffset(uint32_t address);
// NEU: functionCode als Parameter
void encodeTransmission(int address, char* message, uint32_t* out, FunctionCode functionCode);
// NEU: functionCode als Parameter
size_t messageLength(int address, int numChars, FunctionCode functionCode); 
size_t pcmTransmissionLength(uint32_t sampleRate, uint32_t baudRate, size_t transmissionLength);
void pcmEncodeTransmission(uint32_t sampleRate, uint32_t baudRate, uint32_t* transmission, size_t transmissionLength, uint8_t* out);


// =========================================================
// FUNKTION DEFINITIONEN
// =========================================================

/**
 * Calculate the CRC error checking code for the given word.
 */
uint32_t crc(uint32_t inputMsg) {
    uint32_t denominator = CRC_GENERATOR << 20;
    uint32_t msg = inputMsg << CRC_BITS;

    for (int column = 0; column <= 20; column++) {
        int msgBit = (msg >> (30 - column)) & 1;
        if (msgBit != 0) {
            msg ^= denominator;
        }
        denominator >>= 1;
    }
    return msg & 0x3FF;
}

/**
 * Calculates the even parity bit for a message.
 */
uint32_t parity(uint32_t x) {
    uint32_t p = 0;
    for (int i = 0; i < 32; i++) {
        p ^= (x & 1);
        x >>= 1;
    }
    return p;
}

/**
 * Encodes a 21-bit message by calculating and adding a CRC code and parity bit.
 */
uint32_t encodeCodeword(uint32_t msg) {
    uint32_t fullCRC = (msg << CRC_BITS) | crc(msg);
    uint32_t p = parity(fullCRC);
    return (fullCRC << 1) | p;
}

/**
 * ASCII encode a null-terminated string as a series of codewords, written
 * to (*out). Returns the number of codewords written.
 */
uint32_t encodeASCII(uint32_t initial_offset, char* str, uint32_t* out) {
    uint32_t numWordsWritten = 0;
    uint32_t currentWord = 0;
    uint32_t currentNumBits = 0;
    uint32_t wordPosition = initial_offset;

    while (*str != 0) {
        unsigned char c = *str;
        str++;
        for (int i = 0; i < TEXT_BITS_PER_CHAR; i++) {
            currentWord <<= 1;
            currentWord |= (c >> i) & 1;
            currentNumBits++;
            if (currentNumBits == TEXT_BITS_PER_WORD) {
                *out = encodeCodeword(currentWord | FLAG_MESSAGE);
                out++;
                currentWord = 0;
                currentNumBits = 0;
                numWordsWritten++;

                wordPosition++;
                if (wordPosition == BATCH_SIZE) {
                    *out = SYNC;
                    out++;
                    numWordsWritten++;
                    wordPosition = 0;
                }
            }
        }
    }

    //Write remainder of message
    if (currentNumBits > 0) {
        currentWord <<= 20 - currentNumBits;
        *out = encodeCodeword(currentWord | FLAG_MESSAGE);
        out++;
        numWordsWritten++;

        wordPosition++;
        if (wordPosition == BATCH_SIZE) {
            *out = SYNC;
            out++;
            numWordsWritten++;
            wordPosition = 0;
        }
    }

    return numWordsWritten;
}

/**
 * Calculates the number of words which must precede the address word.
 */
uint32_t addressOffset(uint32_t address) {
    return (address & 0x7) * FRAME_SIZE;
}

/**
 * Encode a full POCSAG transmission with a specified function code.
 * (Funktion geändert: Nimmt jetzt FunctionCode entgegen)
 */
void encodeTransmission(int address, char* message, uint32_t* out, FunctionCode functionCode) {

    //Encode preamble
    for (int i = 0; i < PREAMBLE_LENGTH / 32; i++) {
        *out = 0xAAAAAAAA;
        out++;
    }

    uint32_t* start = out;

    //Sync
    *out = SYNC;
    out++;

    //Write out padding before address word
    int prefixLength = addressOffset(address);
    for (int i = 0; i < prefixLength; i++) {
        *out = IDLE;
        out++;
    }

    // Write address word. Der Function Code wird hier eingefügt.
    *out = encodeCodeword( ((address >> 3) << 2) | functionCode);
    out++;

    //Encode the message itself
    out += encodeASCII(addressOffset(address) + 1, message, out);


    //Finally, write an IDLE word indicating the end of the message
    *out = IDLE;
    out++;
    
    //Pad out the last batch with IDLE
    size_t written = out - start;
    size_t padding = (BATCH_SIZE + 1) - written % (BATCH_SIZE + 1);
    for (int i = 0; i < padding; i++) {
        *out = IDLE;
        out++;
    }
}

/**
 * Calculates the length in words of a POCSAG message.
 * (Funktion geändert: Nimmt jetzt FunctionCode entgegen, beeinflusst aber die Logik nicht)
 */
size_t messageLength(int address, int numChars, FunctionCode functionCode) {
    size_t numWords = 0;

    //Padding before address word.
    numWords += addressOffset(address);

    //Address word itself
    numWords++;

    //numChars * 7 bits per character / 20 bits per word, rounding up
    numWords += (numChars * TEXT_BITS_PER_CHAR + (TEXT_BITS_PER_WORD - 1))
                             / TEXT_BITS_PER_WORD;

    //Idle word representing end of message
    numWords++;

    //Pad out last batch with idles
    numWords += BATCH_SIZE - (numWords % BATCH_SIZE);

    //Batches consist of 16 words each and are preceded by a sync word.
    numWords += numWords / BATCH_SIZE;

    //Preamble
    numWords += PREAMBLE_LENGTH / 32;

    return numWords;
}

/**
 * Calculates the length of the PCM transmission.
 */
size_t pcmTransmissionLength(
        uint32_t sampleRate,
        uint32_t baudRate,
        size_t transmissionLength) {
    return transmissionLength * 32 * sampleRate / baudRate * 2;
}

/**
 * PCM-encodes the transmission for SDR use.
 */
void pcmEncodeTransmission(
        uint32_t sampleRate,
        uint32_t baudRate,
        uint32_t* transmission,
        size_t transmissionLength,
        uint8_t* out) {

    int repeatsPerBit = SYMRATE / baudRate;
    int16_t* samples =
        (int16_t*) malloc(sizeof(int16_t) * transmissionLength * 32 * repeatsPerBit);

    int16_t* psamples = samples;
    for (size_t i = 0; i < transmissionLength; i++) {
        uint32_t val = *(transmission + i);
        for (int bitNum = 0; bitNum < 32; bitNum++) {
            int bit = (val >> (31 - bitNum)) & 1;
            int16_t sample;
            if (bit == 0) {
                sample = 32767 / 2;
            } else {
                sample = -32767 / 2;
            }
            for (int r = 0; r < repeatsPerBit; r++) {
                *psamples = sample;
                psamples++;
            }
        }
    }

    //Resample to 22050 sample rate
    size_t outputSize =
        pcmTransmissionLength(sampleRate, baudRate, transmissionLength);
    for (size_t i = 0; i < outputSize; i += 2) {
        int16_t inSample = *(samples + (i / 2) * SYMRATE / sampleRate);
        
        //Write little-endian
        *(out + i + 0) = (inSample & 0xFF);
        *(out + i + 1) = ((inSample >> 8) & 0xFF);
    }

    free(samples);
}


// =========================================================
// MAIN FUNKTION
// =========================================================

int main() {
    //Read in lines from STDIN.
    //Lines are in the format of address:message OR address:function:message
    char line[65536];
    srand(time(NULL));
    for (;;) {

        if (fgets(line, sizeof(line), stdin) == NULL) {
            //Exit on EOF
            return 0;
        }

        // --- Bereinigung der Eingabe
        size_t line_length = strlen(line);
        if (line_length == 0) {
            continue;
        }

        if (line[line_length - 1] == '\n') {
            line_length--;
            line[line_length] = 0;
        }
        if (line[line_length - 1] == '\r') {
            line_length--;
            line[line_length] = 0;
        }

        // --- Parsing
        size_t colonIndex1 = 0;
        size_t colonIndex2 = 0;
        uint32_t colonCount = 0;
        
        for (size_t i = 0; i < line_length; i++) {
            if (line[i] == ':') {
                colonCount++;
                if (colonCount == 1) {
                    colonIndex1 = i;
                } else if (colonCount == 2) {
                    colonIndex2 = i;
                    break; 
                }
            }
        }

        if (colonCount == 0) {
            fprintf(stderr, "Malformed Line: Missing colon separator(s)!\n");
            return 1;
        }

        uint32_t address = 0;
        FunctionCode functionCode = FLAG_FUNC_3;
        char* message = NULL;

        // Fall 1: ADRESSE:NACHRICHT (Ein Doppelpunkt)
        if (colonCount == 1) {
            address = (uint32_t) strtol(line, NULL, 10);
            message = line + colonIndex1 + 1;
            functionCode = FLAG_FUNC_3; // Standard: Alpha (3)
            
        // Fall 2: ADRESSE:FUNKTION:NACHRICHT (Zwei Doppelpunkte)
        } else if (colonCount == 2) {
            // Adresse parsen
            line[colonIndex1] = 0; 
            address = (uint32_t) strtol(line, NULL, 10);
            line[colonIndex1] = ':'; 

            // Funktion parsen
            line[colonIndex2] = 0; 
            char* funcStr = line + colonIndex1 + 1;
            uint32_t funcNum = (uint32_t) strtol(funcStr, NULL, 10);
            line[colonIndex2] = ':'; 
            
            if (funcNum > 3) {
                 fprintf(stderr, "Invalid Function: %u. Must be between 0 and 3.\n", funcNum);
                 return 1;
            }
            functionCode = funcNum;

            // Nachricht parsen
            message = line + colonIndex2 + 1;
        } else {
             fprintf(stderr, "Malformed Line: Too many colons! Expected ADDR:MSG or ADDR:FUNC:MSG.\n");
             return 1;
        }

        // Adressprüfung
        if (address > 2097151) {
            fprintf(stderr, "Address exceeds 21 bits: %u\n", address);
            return 1;
        }

        // --- Kodierung und Ausgabe
        // Korrektur: Variable umbenannt, um Kollision zu vermeiden
        size_t requiredMessageLength = messageLength(address, strlen(message), functionCode);

        uint32_t* transmission =
             (uint32_t*) malloc(sizeof(uint32_t) * requiredMessageLength);

        // NEU: functionCode wird übergeben
        encodeTransmission(address, message, transmission, functionCode);

        size_t pcmLength =
             pcmTransmissionLength(SAMPLE_RATE, BAUD_RATE, requiredMessageLength);

        uint8_t* pcm =
             (uint8_t*) malloc(sizeof(uint8_t) * pcmLength);

        pcmEncodeTransmission(
                 SAMPLE_RATE, BAUD_RATE, transmission, requiredMessageLength, pcm);

        //Write as series of little endian 16 bit samples
        fwrite(pcm, sizeof(uint8_t), pcmLength, stdout);

        free(transmission);
        free(pcm);

        // --- Stille generieren
        size_t silenceLength = rand() % (SAMPLE_RATE * (MAX_DELAY - MIN_DELAY)) + MIN_DELAY;
        uint16_t* silence =
             (uint16_t*) malloc(sizeof(uint16_t) * silenceLength);
        bzero(silence, sizeof(uint16_t) * silenceLength);
        fwrite(silence, sizeof(uint16_t), silenceLength, stdout);
        free(silence);
    }
}