import { useEffect, useState, CSSProperties } from "react";
import r2wc from "@r2wc/react-to-web-component";
import { getAssetUrl } from "@frc-web-components/app";
import './MyElement.css';

function MyElement({ count = 0, blah = "blah" }: { count: number, blah: string }) {
  const [currentCount, setCount] = useState(0);

  useEffect(() => {
    setCount(count);
  }, [count]);

  const styles: CSSProperties = {
    background: 'var(--my-react-element-background, cadetblue)',
    color: 'var(--my-react-element-color, black)',
    border: 'none',
    borderRadius: '3px',
    padding: "8px",
    display: "inline-flex",
    alignItems: "center",
    gap: "8px",
  };

  return (
    <button
      style={styles}
      onClick={() => {
        setCount(currentCount + 1);
      }}
    >
      <img src={getAssetUrl("party.svg")} alt="party time" />
      Party Guests: {currentCount} {blah}
    </button>
  );
}

const WebApp = r2wc(MyElement, {
  props: {
    count: "number",
    blah: "string",
  },
});

customElements.define("my-react-element", WebApp);

export default MyElement;
