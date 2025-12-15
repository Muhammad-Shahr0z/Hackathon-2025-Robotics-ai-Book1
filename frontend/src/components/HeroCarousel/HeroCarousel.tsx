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
           <div className={styles.cardLogo} aria-hidden="true">
  <svg
    width="64"
    height="64"
    viewBox="0 0 64 64"
    fill="none"
    xmlns="http://www.w3.org/2000/svg"
  >
    {/* Head */}
    <rect x="4" y="8" width="56" height="48" rx="10" fill="#A0D8FF" />
    
    {/* Top antenna */}
    <circle cx="32" cy="4" r="4" fill="#FF3B3B" />
    <rect x="30" y="4" width="4" height="8" fill="#007BFF" />
    
    {/* Eyes */}
    <circle cx="20" cy="28" r="8" fill="#005BBB" stroke="#FF3B3B" strokeWidth="2"/>
    <circle cx="44" cy="28" r="8" fill="#005BBB" stroke="#FF3B3B" strokeWidth="2"/>
    
    {/* Eye inner lines */}
    <line x1="16" y1="28" x2="24" y2="28" stroke="#FFFFFF" strokeWidth="2"/>
    <line x1="20" y1="24" x2="20" y2="32" stroke="#FFFFFF" strokeWidth="2"/>
    <line x1="40" y1="28" x2="48" y2="28" stroke="#FFFFFF" strokeWidth="2"/>
    <line x1="44" y1="24" x2="44" y2="32" stroke="#FFFFFF" strokeWidth="2"/>
    
    {/* Nose */}
    <polygon points="30,36 34,36 32,42" fill="#007BFF" />
    
    {/* Mouth */}
    <rect x="22" y="44" width="20" height="6" rx="2" fill="#FFFFFF" stroke="#3B0080" strokeWidth="2" />
    
    {/* Cheeks / side panels */}
    <rect x="0" y="28" width="4" height="16" fill="#FF3B3B" rx="1" />
    <rect x="60" y="28" width="4" height="16" fill="#FF3B3B" rx="1" />
  </svg>
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
