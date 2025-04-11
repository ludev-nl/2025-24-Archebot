import dynamic from 'next/dynamic';

const Map = dynamic(() => import('./map-with-bounding-box'), {
  ssr: false,
});

export default Map;