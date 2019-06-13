/**
 * Copyright (c) 2017-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

const React = require('react');

class Footer extends React.Component {
  docUrl(doc, language) {
    const baseUrl = this.props.config.baseUrl;
    const docsUrl = this.props.config.docsUrl;
    const docsPart = `${docsUrl ? `${docsUrl}/` : ''}`;
    const langPart = `${language ? `${language}/` : ''}`;
    return `${baseUrl}${docsPart}${langPart}${doc}`;
  }

  pageUrl(doc, language) {
    const baseUrl = this.props.config.baseUrl;
    return baseUrl + (language ? `${language}/` : '') + doc;
  }

  render() {
    return (
      <footer className="nav-footer" id="footer">
        <section className="sitemap">
          <a href={this.props.config.baseUrl} className="nav-home">
            {this.props.config.footerIcon && (
              <img
                src={this.props.config.baseUrl + this.props.config.footerIcon}
                alt={this.props.config.title}
                width="66"
                height="58"
              />
            )}
          </a>
          <div>
            <h5>Docs</h5>
            <a href={this.docUrl('overview.html')}>
              Getting Started
            </a>
            <a href={this.docUrl('calibration.html')}>
              Examples
            </a>
            <a href={this.docUrl('datasets.html')}>
              Datasets
            </a>
            <a href={this.docUrl('new_robot_support.html')}>
              Help and Support
            </a>
          </div>
          <div>
            <h5>More</h5>
            <a href="https://github.com/facebookresearch/pyrobot">GitHub</a>
	    <a class="github-button" href="https://github.com/facebookresearch/pyrobot" data-icon="octicon-star" data-show-count="true" aria-label="Star facebookresearch/pyrobot on GitHub">Star</a>
            <a href={this.docUrl('contact.html')}>
              Contact
            </a>
          </div>
        </section>
      </footer>
    );
  }
}

module.exports = Footer;
