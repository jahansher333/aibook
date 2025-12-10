import type {ReactNode} from 'react';
import {useState, useEffect} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const [typedText, setTypedText] = useState('');
  const fullText = 'Build Intelligent Robots with AI';

  useEffect(() => {
    let index = 0;
    const timer = setInterval(() => {
      if (index <= fullText.length) {
        setTypedText(fullText.slice(0, index));
        index++;
      } else {
        clearInterval(timer);
      }
    }, 100);
    return () => clearInterval(timer);
  }, []);

  return (
    <header className={styles.heroBanner}>
      <div className={styles.particlesBackground}>
        {[...Array(20)].map((_, i) => (
          <div key={i} className={styles.particle} style={{
            left: `${Math.random() * 100}%`,
            animationDelay: `${Math.random() * 5}s`,
            animationDuration: `${5 + Math.random() * 10}s`
          }} />
        ))}
      </div>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <div className={styles.badge}>
              <span className={styles.badgeIcon}>⚡</span>
              <span>Free & Open Source</span>
            </div>
            <Heading as="h1" className={styles.heroTitle}>
              Master Physical AI &<br/>
              <span className={styles.gradientText}>{typedText}</span>
              <span className={styles.cursor}>|</span>
            </Heading>
            <p className={styles.heroSubtitle}>
              From zero to hero in 13 weeks. Learn ROS 2, Isaac Sim, and Vision-Language-Action models
              with hands-on projects. Deploy to real robots or cloud GPUs.
            </p>
            <div className={styles.heroButtons}>
              <Link
                className={clsx('button button--primary button--lg', styles.ctaButton)}
                to="/docs/intro">
                <span>Start Free Course</span>
                <span className={styles.arrow}>→</span>
              </Link>
              <Link
                className={clsx('button button--outline button--lg', styles.secondaryButton)}
                to="/docs/ch01-physical-ai-intro/ch01">
                <span>Preview Chapter 1</span>
              </Link>
            </div>
            <div className={styles.socialProof}>
              <div className={styles.avatars}>
                <div className={styles.avatar}>👨‍💻</div>
                <div className={styles.avatar}>👩‍🔬</div>
                <div className={styles.avatar}>👨‍🎓</div>
                <div className={styles.avatar}>👩‍💼</div>
              </div>
              <p className={styles.proofText}>Join thousands learning robotics</p>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <div className={styles.robotContainer}>
              <div className={styles.robotMain}>🤖</div>
              <div className={styles.orbits}>
                <div className={clsx(styles.orbit, styles.orbit1)}>
                  <div className={styles.orbitItem}>🦾</div>
                </div>
                <div className={clsx(styles.orbit, styles.orbit2)}>
                  <div className={styles.orbitItem}>🎮</div>
                </div>
                <div className={clsx(styles.orbit, styles.orbit3)}>
                  <div className={styles.orbitItem}>🗣️</div>
                </div>
              </div>
            </div>
            <div className={styles.techStack}>
              <div className={styles.techItem}>ROS 2</div>
              <div className={styles.techItem}>Isaac Sim</div>
              <div className={styles.techItem}>GPT-4</div>
              <div className={styles.techItem}>Gazebo</div>
              <div className={styles.techItem}>Unity</div>
            </div>
          </div>
        </div>
      </div>
      <div className={styles.waveBottom}>
        <svg viewBox="0 0 1440 120" xmlns="http://www.w3.org/2000/svg">
          <path fill="var(--ifm-background-color)" d="M0,64L80,69.3C160,75,320,85,480,80C640,75,800,53,960,48C1120,43,1280,53,1360,58.7L1440,64L1440,120L1360,120C1280,120,1120,120,960,120C800,120,640,120,480,120C320,120,160,120,80,120L0,120Z"></path>
        </svg>
      </div>
    </header>
  );
}

function CourseTimeline() {
  const weeks = [
    { title: 'Weeks 1-3: Foundations', desc: 'Physical AI, ROS 2, URDF modeling', icon: '📚' },
    { title: 'Weeks 4-6: Simulation', desc: 'Gazebo, Unity, Isaac Sim', icon: '🎮' },
    { title: 'Weeks 7-9: Advanced AI', desc: 'VLA models, kinematics, locomotion', icon: '🧠' },
    { title: 'Weeks 10-13: Integration', desc: 'Hardware deployment & capstone', icon: '🚀' },
  ];

  return (
    <section className={styles.timeline}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          13-Week Learning Journey
        </Heading>
        <div className={styles.timelineGrid}>
          {weeks.map((week, idx) => (
            <div key={idx} className={styles.timelineCard}>
              <div className={styles.timelineIcon}>{week.icon}</div>
              <h3 className={styles.timelineTitle}>{week.title}</h3>
              <p className={styles.timelineDesc}>{week.desc}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function HardwareTiers() {
  const tiers = [
    { name: 'Economy', cost: '$700', desc: 'Jetson Orin Nano + RealSense', icon: '💰', color: 'green' },
    { name: 'Proxy', cost: '$3,850', desc: 'Unitree Go2 Quadruped', icon: '🐕', color: 'blue' },
    { name: 'Cloud', cost: '$205/qtr', desc: 'AWS EC2 GPU instances', icon: '☁️', color: 'purple' },
  ];

  return (
    <section className={styles.hardware}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          Hardware Options
        </Heading>
        <p className={styles.sectionSubtitle}>Choose your learning path based on budget and goals</p>
        <div className={styles.hardwareGrid}>
          {tiers.map((tier, idx) => (
            <div key={idx} className={clsx(styles.hardwareCard, styles[`tier${tier.color}`])}>
              <div className={styles.hardwareIcon}>{tier.icon}</div>
              <h3 className={styles.hardwareName}>{tier.name}</h3>
              <div className={styles.hardwareCost}>{tier.cost}</div>
              <p className={styles.hardwareDesc}>{tier.desc}</p>
            </div>
          ))}
        </div>
        <div className={styles.hardwareCta}>
          <Link to="/docs/hardware-requirements" className="button button--outline button--lg">
            Compare Full Specs →
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Interactive Robotics Textbook"
      description="Learn ROS 2, Isaac Sim, and Vision-Language-Action models for humanoid robots. 13 comprehensive chapters with hands-on labs and interactive assessments.">
      <HomepageHeader />
      <main>
        <CourseTimeline />
        <HardwareTiers />
      </main>
    </Layout>
  );
}
