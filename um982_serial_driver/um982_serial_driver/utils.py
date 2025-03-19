def check_crc(nmea_sentence):
    # create crc_table
    def crc_table():
        table = []
        for i in range(256):
            crc = i
            for j in range(8, 0, -1):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xEDB88320
                else:
                    crc >>= 1
            table.append(crc)
        return table

    # Function to calculate CRC using the provided algorithm and the generated table
    def calculate_crc32(data):
        table = crc_table()
        crc = 0
        for byte in data:
            crc = table[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    # main check_crc
    try:
        sentence, crc = nmea_sentence[1:].split("*")
        # print(sentence)
    except:
        return False
    ret =  int(crc,16) == calculate_crc32(sentence.encode())
    return ret


def check_checksum(nmea_sentence):
    try:
        sentence, checksum = nmea_sentence[1:].split("*")
    except:
        return False
    calculated_checksum = 0
    for char in sentence:
        calculated_checksum ^= ord(char)
    return calculated_checksum == int(checksum, 16)


def msg_seperate(msg: str):
    return msg[1 : msg.find("*")].split(",")


def determine_utm_zone_and_hemisphere(lat, lon):
    zone_number = int((lon + 180) / 6) + 1
    north = lat >= 0

    return zone_number, north
