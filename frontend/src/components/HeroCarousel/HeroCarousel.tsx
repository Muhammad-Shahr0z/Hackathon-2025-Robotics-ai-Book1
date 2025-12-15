import React from 'react';
import styles from './HeroCarousel.module.css';

const cards = [
  { title: 'Generative Models', desc: 'GANs and diffusion models for data and images.' },
  { title: 'Reinforcement Learning', desc: 'Train agents with rewards and environments.' },
  { title: 'Computer Vision', desc: 'Detection, segmentation, and perception for robots.' },
  { title: 'Natural Language Processing', desc: 'Transformers, embeddings, and agents.' },
  { title: 'Robotics Control', desc: 'Kinematics, dynamics, and learning-based control.' },
  { title: 'Simulation & Digital Twins', desc: 'Simulate worlds for safe robot training.' },
];

export default function HeroCarousel() {
 
  const loop = [...cards, ...cards];
  return (
    <section className={styles.carousel} aria-hidden="false">
      <div className={styles.inner}>
        <div className={styles.track}>
        {loop.map((c, i) => (
          <article key={`${c.title}-${i}`} className={styles.card}>
            <div className={styles.cardLogo} aria-hidden>
             <img src="/img/robologo.png" alt="robologo" />
            </div>
            <h4 className={styles.cardTitle}>{c.title}</h4>
            <p className={styles.cardDesc}>{c.desc}</p>
          </article>
        ))}
        </div>
      </div>
    </section>
  );
}
