function Decode(port, bytes) {
  if (port == 1) {
    return {
      last: (((bytes[0] << 2) | (bytes[1] >> 6)) &0x3FF) / 10,
      avg: (((bytes[1] << 4) | (bytes[2] >> 4)) &0x3FF) / 10,
      min: (((bytes[2] << 6) | (bytes[3] >> 2)) &0x3FF) / 10,
      max: (((bytes[3] << 8) | (bytes[4])) &0x3FF) / 10,
      battery: Number((((bytes[5]) + 300) / 100).toFixed(2)),
      deviceType: "water-level"
    };
  } else if (port == 2) {
    var error = ((bytes[0] >> 6) + 1) * -1;
    return {
      error: error,
      count: (((bytes[0]) & 0x3F) << 8) | (bytes[1]),
      deviceType: "water-level-error"
    }
  }
}