import type { ReactNode } from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import MatrixCanvas from "@site/src/components/MatrixCanvas";

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Physical AI & Humanoid Robotics Textbook"
    >
      <MatrixCanvas opacity={0.12} />

      <div
        style={{
          position: "relative",
          zIndex: 10,
          textAlign: "center",
          padding: "120px 20px 60px",
          color: "white",
        }}
      >
        <h1 style={{ fontSize: "3rem", marginBottom: "0.5rem" }}>
          Physical AI & Humanoid Robotics
        </h1>
        <h2 style={{ fontSize: "1.5rem", opacity: 0.85 }}>
          Embodied Intelligence Textbook
        </h2>
      </div>

      <main />
    </Layout>
  );
}
