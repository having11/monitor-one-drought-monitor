import Particle from 'particle:core';
import { encode } from 'vendor:cbor-x';
import { base64Encode } from 'particle:encoding';

export default function reformat({ event }) {
  let data;
  try {
	  data = JSON.parse(event.eventData);
  } catch (err) {
    console.error("Invalid JSON", event.eventData);
    throw err;
  }

  const cborBuffer = encode(data.loc.weather);
  const base64Data = base64Encode(Array.from(cborBuffer));

  console.log(`Sending base64 drought data: ${base64Data}`);

  Particle.publish("drought-cbor", base64Data, { productId: event.productId });
}
