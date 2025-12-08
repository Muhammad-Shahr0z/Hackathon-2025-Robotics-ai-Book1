import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/Hackathon-2025-Robotics-ai-Book1/markdown-page',
    component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/markdown-page', 'aa1'),
    exact: true
  },
  {
    path: '/Hackathon-2025-Robotics-ai-Book1/docs',
    component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs', '005'),
    routes: [
      {
        path: '/Hackathon-2025-Robotics-ai-Book1/docs',
        component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs', '3f1'),
        routes: [
          {
            path: '/Hackathon-2025-Robotics-ai-Book1/docs',
            component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs', '915'),
            routes: [
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/', '38c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-1/',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-1/', '5c4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-1/chapter-1',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-1/chapter-1', '73a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-1/chapter-2',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-1/chapter-2', 'b7c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-1/chapter-3',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-1/chapter-3', '14e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-2/',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-2/', 'a8a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-2/chapter-4',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-2/chapter-4', '3f6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-2/chapter-5',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-2/chapter-5', '555'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-2/chapter-6',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-2/chapter-6', '6ec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-3/',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-3/', 'aa4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-3/chapter-7',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-3/chapter-7', '2dd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-3/chapter-8',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-3/chapter-8', '17d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-3/chapter-9',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-3/chapter-9', 'de1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-4/',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-4/', '43b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-4/chapter-10',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-4/chapter-10', '643'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-4/chapter-11',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-4/chapter-11', '003'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-4/chapter-12',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-4/chapter-12', '581'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-5/',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-5/', '2d7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-2025-Robotics-ai-Book1/docs/module-5/capstone',
                component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/docs/module-5/capstone', 'c53'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/Hackathon-2025-Robotics-ai-Book1/',
    component: ComponentCreator('/Hackathon-2025-Robotics-ai-Book1/', '689'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
