#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <time.h>

// =========================================================
// CONSTANTS AND TYPES
// =========================================================

// Check out https://en.wikipedia.org/wiki/POCSAG
// Also see http://www.itu.int/dms_pubrec/itu-r/rec/m/R-REC-M.584-2-199711-I!!PDF-E.pdf
// They'll be handy when trying to understand this stuff.

// The sync word exists at the start of every batch.
// A batch is 16 words, so a sync word occurs every 16 data words.
#define SYNC 0x7CD215D8

// The idle word is used as padding before the address word, and at the end
// of a message to indicate that the message is finished.
#define IDLE 0x7A89C197

// One frame consists of a pair of two words
#define FRAME_SIZE 2

// One batch consists of 8 frames, or 16 words
#define BATCH_SIZE 16

// The preamble comes before a message, and is a series of alternating
// 1,0,1,0... bits, for at least 576 bits. It exists to allow the receiver
// to synchronize with the transmitter
#define PREAMBLE_LENGTH 576

// These bits appear as the first bit of a word, 0 for an address word and
// one for a data word
#define FLAG_ADDRESS 0x000000
#define FLAG_MESSAGE 0x100000

// Each data word can contain 20 bits of text information. Each character is
// 7 bits wide, ASCII encoded.
#define TEXT_BITS_PER_WORD 20
#define TEXT_BITS_PER_CHAR 7
#define CRC_BITS 10
#define CRC_GENERATOR 0b11101101001

// The last two bits of an address word's data represent the data type
// (Function Codes 0-3).
#define FLAG_FUNC_0 0x0 // Alert/Numeric
#define FLAG_FUNC_1 0x1 // Numeric
#define FLAG_FUNC_2 0x2 // Numeric
#define FLAG_FUNC_3 0x3 // Alpha (Text)
typedef uint32_t FunctionCode;

// PCM/Audio Constants
#define SYMRATE 38400
#define SAMPLE_RATE 22050
#define BAUD_RATE 512
#define MIN_DELAY 1
#define MAX_DELAY 10

// =========================================================
// FUNCTION PROTOTYPES
// =========================================================

uint32_t crc(uint32_t inputMsg);
uint32_t parity(uint32_t x);
uint32_t encodeCodeword(uint32_t msg);
uint32_t encodeASCII(uint32_t initial_offset, char* str, uint32_t* out);
uint32_t addressOffset(uint32_t address);
// NEW: Added FunctionCode parameter to encode the correct message type.
void encodeTransmission(int address, char* message, uint32_t* out, FunctionCode functionCode);
// NEW: Added FunctionCode parameter, though it does not change the message length calculation for ASCII messages.
size_t messageLength(int address, int numChars, FunctionCode functionCode); 
size_t pcmTransmissionLength(uint32_t sampleRate, uint32_t baudRate, size_t transmissionLength);
void pcmEncodeTransmission(uint32_t sampleRate, uint32_t baudRate, uint32_t* transmission, size_t transmissionLength, uint8_t* out);


// =========================================================
// FUNCTION DEFINITIONS
// =========================================================

/**
 * Calculate the CRC error checking code for the given word.
 * Messages use a 10 bit CRC computed from the 21 data bits.
 */
uint32_t crc(uint32_t inputMsg) {
    //Align MSB of denominatorerator with MSB of message
    uint32_t denominator = CRC_GENERATOR << 20;

    //Message is right-padded with zeroes to the message length + crc length
    uint32_t msg = inputMsg << CRC_BITS;

    //We iterate until denominator has been right-shifted back to it's original value.
    for (int column = 0; column <= 20; column++) {
        //Bit for the column we're aligned to
        int msgBit = (msg >> (30 - column)) & 1;

        //If the current bit is zero, we don't modify the message this iteration
        if (msgBit != 0) {
            //While we would normally subtract in long division, we XOR here.
            msg ^= denominator;
        }

        //Shift the denominator over to align with the next column
        denominator >>= 1;
    }

    //At this point 'msg' contains the CRC value we've calculated
    return msg & 0x3FF;
}

/**
 * Calculates the even parity bit for a message.
 * If the number of bits in the message is even, return 0, else return 1.
 */
uint32_t parity(uint32_t x) {
    //Our parity bit
    uint32_t p = 0;

    //We xor p with each bit of the input value.
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
    //Number of words written to *out
    uint32_t numWordsWritten = 0;

    //Data for the current word we're writing
    uint32_t currentWord = 0;

    //Nnumber of bits we've written so far to the current word
    uint32_t currentNumBits = 0;

    //Position of current word in the current batch
    uint32_t wordPosition = initial_offset;

    while (*str != 0) {
        unsigned char c = *str;
        str++;
        //Encode the character bits backwards (LSB first)
        for (int i = 0; i < TEXT_BITS_PER_CHAR; i++) {
            currentWord <<= 1;
            currentWord |= (c >> i) & 1;
            currentNumBits++;
            if (currentNumBits == TEXT_BITS_PER_WORD) {
                //Add the MESSAGE flag to our current word and encode it.
                *out = encodeCodeword(currentWord | FLAG_MESSAGE);
                out++;
                currentWord = 0;
                currentNumBits = 0;
                numWordsWritten++;

                wordPosition++;
                if (wordPosition == BATCH_SIZE) {
                    //We've filled a full batch, time to insert a SYNC word
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
        //Pad out the word to 20 bits with zeroes
        currentWord <<= 20 - currentNumBits;
        *out = encodeCodeword(currentWord | FLAG_MESSAGE);
        out++;
        numWordsWritten++;

        wordPosition++;
        if (wordPosition == BATCH_SIZE) {
            //We've filled a full batch, time to insert a SYNC word
            *out = SYNC;
            out++;
            numWordsWritten++;
            wordPosition = 0;
        }
    }

    return numWordsWritten;
}

/**
 * An address of 21 bits, but only 18 of those bits are encoded in the address
 * word itself. The remaining 3 bits are derived from which frame in the batch
 * is the address word. This calculates the number of words (not frames!)
 * which must precede the address word. These words will be filled with the idle value.
 */
uint32_t addressOffset(uint32_t address) {
    return (address & 0x7) * FRAME_SIZE;
}

/**
 * Encode a full POCSAG transmission addressed to (address) with the given Function Code.
 * The Function Code determines the message type (e.g., text, numeric).
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

    // Write address word. The Function Code is inserted here (bits 1-2).
    // The 3 least significant bits of the address are dropped, as those are encoded by the
    // word's location (address offset).
    *out = encodeCodeword( ((address >> 3) << 2) | functionCode);
    out++;

    //Encode the message itself
    out += encodeASCII(addressOffset(address) + 1, message, out);


    //Finally, write an IDLE word indicating the end of the message
    *out = IDLE;
    out++;
    
    //Pad out the last batch with IDLE to write multiple of BATCH_SIZE + 1
    //words (+ 1 is there because of the SYNC words)
    size_t written = out - start;
    size_t padding = (BATCH_SIZE + 1) - written % (BATCH_SIZE + 1);
    for (int i = 0; i < padding; i++) {
        *out = IDLE;
        out++;
    }
}

/**
 * Calculates the length in words of a POCSAG message.
 * The FunctionCode is included for consistency but does not affect the length
 * calculation for a standard ASCII/Text message.
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
    //So we add one word for every 16 message words
    numWords += numWords / BATCH_SIZE;

    //Preamble of 576 alternating 1,0,1,0 bits before the message
    numWords += PREAMBLE_LENGTH / 32;

    return numWords;
}

/**
 * Calculates the length of the PCM transmission.
 * (32 bits per word * (sampleRate / baudRate) samples * 2 bytes/sample)
 */
size_t pcmTransmissionLength(
        uint32_t sampleRate,
        uint32_t baudRate,
        size_t transmissionLength) {
    return transmissionLength * 32 * sampleRate / baudRate * 2;
}

/**
 * PCM-encodes the transmission for SDR use.
 * It encodes data at SYMRATE (38400 Hz) and then "resamples" to SAMPLE_RATE (22050 Hz).
 */
void pcmEncodeTransmission(
        uint32_t sampleRate,
        uint32_t baudRate,
        uint32_t* transmission,
        size_t transmissionLength,
        uint8_t* out) {

    //Number of times we need to repeat each bit to achieve SYMRATE
    int repeatsPerBit = SYMRATE / baudRate;

    //Initial buffer for samples before resampling occurs
    int16_t* samples =
        (int16_t*) malloc(sizeof(int16_t) * transmissionLength * 32 * repeatsPerBit);

    //Pointer to samples we can modify in the loop
    int16_t* psamples = samples;
    for (size_t i = 0; i < transmissionLength; i++) {
        uint32_t val = *(transmission + i);
        for (int bitNum = 0; bitNum < 32; bitNum++) {

            //Encode from most significant to least significant bit
            int bit = (val >> (31 - bitNum)) & 1;
            int16_t sample;
            // A negative value represents 1, while a positive value represents 0 (FSK simulation)
            if (bit == 0) {
                sample = 32767 / 2;
            } else {
                sample = -32767 / 2;
            }

            //Repeat as many times as we need for the current baudrate
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
        //Round to closest index in input data which corresponds to output index (nearest neighbor resampling)
        int16_t inSample = *(samples + (i / 2) * SYMRATE / sampleRate);
        
        //Write little-endian
        *(out + i + 0) = (inSample & 0xFF);
        *(out + i + 1) = ((inSample >> 8) & 0xFF);
    }

    //And we're done! Delete our temporary buffer
    free(samples);
}


// =========================================================
// MAIN FUNCTION
// =========================================================

int main() {
    // Read in lines from STDIN.
    // Lines are expected in the format of **address:message** OR **address:function:message**.
    // The default function code is FLAG_FUNC_3 (Alpha/Text).
    char line[65536];
    srand(time(NULL));
    for (;;) {

        if (fgets(line, sizeof(line), stdin) == NULL) {
            //Exit on EOF
            return 0;
        }

        // --- Input Cleaning
        size_t line_length = strlen(line);
        if (line_length == 0) {
            continue;
        }

        // Remove trailing newline and carriage return characters
        if (line[line_length - 1] == '\n') {
            line_length--;
            line[line_length] = 0;
        }
        if (line_length > 0 && line[line_length - 1] == '\r') {
            line_length--;
            line[line_length] = 0;
        }
        
        if (line_length == 0) {
             continue;
        }

        // --- Parsing
        size_t colonIndex1 = 0;
        size_t colonIndex2 = 0;
        uint32_t colonCount = 0;
        
        // Find colon separators to determine input format (ADDR:MSG vs. ADDR:FUNC:MSG)
        for (size_t i = 0; i < line_length; i++) {
            if (line[i] == ':') {
                colonCount++;
                if (colonCount == 1) {
                    colonIndex1 = i;
                } else if (colonCount == 2) {
                    colonIndex2 = i;
                    break; // Stop after finding the second colon
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

        // Case 1: ADDRESS:MESSAGE (One colon)
        if (colonCount == 1) {
            address = (uint32_t) strtol(line, NULL, 10);
            message = line + colonIndex1 + 1;
            functionCode = FLAG_FUNC_3; // Default to Alpha/Text (3)
            
        // Case 2: ADDRESS:FUNCTION:MESSAGE (Two colons)
        } else if (colonCount == 2) {
            // Parse Address: Temporarily null-terminate to parse the address part
            line[colonIndex1] = 0; 
            address = (uint32_t) strtol(line, NULL, 10);
            line[colonIndex1] = ':'; // Restore colon

            // Parse Function: Temporarily null-terminate to parse the function code
            line[colonIndex2] = 0; 
            char* funcStr = line + colonIndex1 + 1;
            uint32_t funcNum = (uint32_t) strtol(funcStr, NULL, 10);
            line[colonIndex2] = ':'; // Restore colon
            
            if (funcNum > 3) {
                 fprintf(stderr, "Invalid Function: %u. Must be between 0 and 3.\n", funcNum);
                 return 1;
            }
            functionCode = funcNum;

            // Message starts after the second colon
            message = line + colonIndex2 + 1;
        } else {
             fprintf(stderr, "Malformed Line: Too many colons! Expected ADDR:MSG or ADDR:FUNC:MSG.\n");
             return 1;
        }

        // Address validation (Largest 21-bit address)
        if (address > 2097151) {
            fprintf(stderr, "Address exceeds 21 bits: %u\n", address);
            return 1;
        }

        // --- Encoding and Output
        // Renamed variable from 'messageLength' to 'requiredMessageLength' to avoid shadowing the function name.
        size_t requiredMessageLength = messageLength(address, strlen(message), functionCode);

        uint32_t* transmission =
             (uint32_t*) malloc(sizeof(uint32_t) * requiredMessageLength);

        // NEW: Passing the determined functionCode to the encoder.
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

        // Generate rand amount of silence (0-value samples).
        // 1-10 seconds
        size_t silenceLength = rand() % (SAMPLE_RATE * (MAX_DELAY - MIN_DELAY)) + MIN_DELAY;

        //Since the values are zero, endianness doesn't matter here
        uint16_t* silence =
             (uint16_t*) malloc(sizeof(uint16_t) * silenceLength);
        bzero(silence, sizeof(uint16_t) * silenceLength);
        fwrite(silence, sizeof(uint16_t), silenceLength, stdout);
        free(silence);
    }
}