import dynamic from 'next/dynamic';

const Map = dynamic(() => import('./map-with-box'), {
  ssr: false,
});

export default Map;