/**
 * Copyright (c) 2017-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

const React = require('react');

const CompLibrary = require('../../core/CompLibrary.js');

const MarkdownBlock = CompLibrary.MarkdownBlock; /* Used to read markdown */
const Container = CompLibrary.Container;
const GridBlock = CompLibrary.GridBlock;

class HomeSplash extends React.Component {

  render() {
    const {siteConfig, language = ''} = this.props;
    const {baseUrl, docsUrl} = siteConfig;
    const docsPart = `${docsUrl ? `${docsUrl}/` : ''}`;
    const langPart = `${language ? `${language}/` : ''}`;
    const docUrl = doc => `${baseUrl}${docsPart}${langPart}${doc}`;

    const SplashContainer = props => (
      <div className="homeContainer">
        <div className="homeSplashFade">
          <div className="wrapper homeWrapper">{props.children}</div>
        </div>
      </div>
    );

    const Logo = props => (
      <div className="projectLogo">
        <img src={props.img_src} alt="Project Logo" />
      </div>
    );

    const ProjectTitle = () => (
      <h2 className="projectTitle">
        {siteConfig.title}
        <small>{siteConfig.tagline}</small>
      </h2>
    );

    const PromoSection = props => (
      <div className="section promoSection">
        <div className="promoRow">
          <div className="pluginRowBlock">{props.children}</div>
        </div>
      </div>
    );

    const Button = props => (
      <div className="pluginWrapper buttonWrapper">
        <a className="button" href={props.href} target={props.target}>
          {props.children}
        </a>
      </div>
    );

    return (
      <SplashContainer>
        <div className="inner">
          <ProjectTitle siteConfig={siteConfig} />
          <PromoSection>
            <Button href={docUrl('overview.html')}>Get Started</Button>
            <Button href="https://github.com/facebookresearch/pyrobot">GitHub</Button>
          </PromoSection>
        </div>
        <Logo img_src={`${baseUrl}img/pyrobot_icon.svg`} />
      </SplashContainer>
    );
  }
}

class Index extends React.Component {
  render() {
    const {config: siteConfig, language = ''} = this.props;
    const {baseUrl} = siteConfig;
    const pageUrl = page => baseUrl + (language ? `${language}/` : '') + page;

    const Block = props => (
      <Container
        padding={['bottom', 'top']}
        id={props.id}
        background={props.background}>
        <GridBlock
          align="center"
          contents={props.children}
          layout={props.layout}
        />
      </Container>
    );

    const FeatureCallout = () => (
      <div
        className="productShowcaseSection"
        style={{textAlign: 'center'}}>
        <h2>What can you do with PyRobot?</h2>

        <div className="row">
          <div className="column">
            <img className="index_vid" src="https://thumbs.gfycat.com/FickleSpeedyChimneyswift-size_restricted.gif" alt="GIF" />
            <br></br>
            <h3>Manipulation</h3>
          </div>
          <div className="column">
            <img className="index_vid" src="https://thumbs.gfycat.com/FinishedWeirdCockerspaniel-size_restricted.gif" alt="GIF" />
            <br></br>
            <h3>Navigation</h3>
          </div>
          <div className="column">
            <img className="index_vid" src="https://thumbs.gfycat.com/WeightyLeadingGrub-size_restricted.gif" alt="GIF" />
            <br></br>
            <h3>Demonstrations</h3>
          </div>
        </div>

      </div>
    );

    const TryOut = () => (
      <Block id="try">
        {[
          {
            content: 'Talk about trying this out',
            image: `${baseUrl}img/pyrobot_icon.svg`,
            imageAlign: 'left',
            title: 'Try it Out',
          },
        ]}
      </Block>
    );

    const Description = () => (
      <Block background="light">
        {[
          {
            content:
              'PyRobot is a Python package for benchmarking and running experiments in robot learning. The goal of this project is \
              to abstract away the low-level controls for individual robots from the high-level motion generation and learning in  \
              an easy-to-use way. Using PyRobot will hence allow you to run robots without having to deal with the robot specific \
              software (like ROS). \
              Currently we support the following robots: [LoCoBot](https://sites.google.com/andrew.cmu.edu/lowcostrobot), [Sawyer](https://www.rethinkrobotics.com/sawyer/). If you would like to support your own robot with PyRobot, please follow the instructions [here](`${docUrl(doc1.html)}`).',
            title: 'What is PyRobot?',
          },
        ]}
      </Block>
    );

    const HtmlDescription = () =>(
      <div 
      background="light"
      className="productShowcaseSection paddingBottom backgroundLight"
      style={{textAlign:'center',marginLeft:'5%',marginRight:'5%'}}>
        <h2>What is PyRobot?</h2>

        PyRobot is an open source robotics research platform. It aims to democratize robotics by reducing the entry barrier. 
        Specifically, we abstract away the low-level controls for individual robots from the high-level motion generation and learning in 
        an easy-to-use way. Using PyRobot will hence allow you to run robots without having to deal with the robot specific
        software (like ROS).

        Currently we support the following robots: <a href="https://sites.google.com/andrew.cmu.edu/lowcostrobot">LoCoBot</a>, <a href="https://www.rethinkrobotics.com/sawyer/">Sawyer</a>. 
        If you would like to support your own robot with PyRobot, please follow the instructions <a href={pageUrl('overview.html')}>here</a>.
      </div>
    );

    const LearnHow = () => (
      <Block background="light">
        {[
          {
            content: 'PyRobot is an open source robotics research platform. It aims to democratize robotics by reducing the entry barrier. Specifically, we abstract away the low-level controls for individual robots from the high-level motion generation and learning in an easy-to-use way. Using PyRobot will hence allow you to run robots without having to deal with the robot specific software (like ROS).',
            title: 'What is PyRobot?',
          },
        ]}
      </Block>
    );

    const Features = () => (
      <Block layout="fourColumn">
        {[
          {
            content: 'This is the content of my feature',
            image: `${baseUrl}img/pyrobot_icon.svg`,
            imageAlign: 'top',
            title: 'Feature One',
          },
          {
            content: 'The content of my second feature',
            image: `${baseUrl}img/pyrobot_icon.svg`,
            imageAlign: 'top',
            title: 'Feature Two',
          },
        ]}
      </Block>
    );

    const Showcase = () => {
      if ((siteConfig.users || []).length === 0) {
        return null;
      }

      const showcase = siteConfig.users
        .filter(user => user.pinned)
        .map(user => (
          <a href={user.infoLink} key={user.infoLink}>
            <img src={user.image} alt={user.caption} title={user.caption} />
          </a>
        ));

      const pageUrl = page => baseUrl + (language ? `${language}/` : '') + page;

      return (
        <div className="productShowcaseSection paddingBottom">
          <h2>Who is Using This?</h2>
          <p>This project is used by all these people</p>
          <div className="logos">{showcase}</div>
          <div className="more-users">
            <a className="button" href={pageUrl('users.html')}>
              More {siteConfig.title} Users
            </a>
          </div>
        </div>
      );
    };

    return (
      <div>
        <HomeSplash siteConfig={siteConfig} language={language} />
        <div className="mainContainer">
          <FeatureCallout />
        </div>
      </div>
    );
  }
}

module.exports = Index;
